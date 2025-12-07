#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from evry_project_plugins.srv import DistanceToFlag


class Robot:
    def __init__(self, robot_name):
        """Robot wrapper: subscribers/publisher + simple helpers."""
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0  # Sonar distance
        self.x, self.y = 0.0, 0.0  # coordinates of the robot
        self.yaw = 0.0  # yaw angle of the robot
        self.robot_name = robot_name

        rospy.Subscriber(
            self.robot_name + "/sensor/sonar_front",
            Range,
            self.callbackSonar,
        )
        rospy.Subscriber(
            self.robot_name + "/odom",
            Odometry,
            self.callbackPose,
        )
        self.cmd_vel_pub = rospy.Publisher(
            self.robot_name + "/cmd_vel", Twist, queue_size=1
        )

    def callbackSonar(self, msg):
        self.sonar = msg.range

    def get_sonar(self):
        return self.sonar

    def callbackPose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = euler_from_quaternion(quaternion_list)
        self.yaw = yaw

    def get_robot_pose(self):
        return self.x, self.y, self.yaw

    def constraint(self, val, min=-2.0, max=2.0):
        if val < min:
            return min
        if val > max:
            return max
        return val

    def set_speed_angle(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear)
        cmd_vel.angular.z = self.constraint(angular, min=-1, max=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            robot_id = int(self.robot_name.split('_')[-1])
            result = service(pose, robot_id)
            return result.distance
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return float("inf")


def run_demo():
    """
    Improved obstacle avoidance with:
      - GO_TO_GOAL
      - AVOID_TURN   : rotate away from goal/obstacle with min angle
      - AVOID_STRAIGHT: drive straight while keeping clearance, then back to GO_TO_GOAL
    """
    '''
        ███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████
    This strategy computes the *actual position of the flag* at startup using the robots pose and its initial 
    distance-to-flag. From that moment, the robot always knows the goals (x, y) location and continuously turns 
    to align its heading toward the goal before moving forward.

    When no obstacle is detected, the robot turns toward the goal direction (if needed) and then drives straight 
    toward it. This ensures that movement is always directed along the shortest geometric path to the destination.

    When an obstacle is detected by the sonar, the robot enters a structured two-step avoidance maneuver:
      1) AVOID_TURN:
           It rotates *away from the goal* by at least a minimal angle, guaranteeing it turns outward and not 
           into narrow gaps between obstacles.
      2) AVOID_STRAIGHT:
           It then drives straight for a short, predefined distance to bypass the obstacle laterally.  
           If the sonar becomes too close again, it adds a small outward turn while moving.

    After completing the bypass, the robot reorients toward the goal and resumes normal motion.  
    ███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████

    '''

    robot_name = rospy.get_param("~robot_name", "robot_1")
    robot = Robot(robot_name)
    print(f"{robot_name}: starting improved avoidance planner")

    # staggered start
    try:
        robot_index = int(robot_name.split('_')[-1])
    except ValueError:
        robot_index = 1
    delay_between = 0.5
    rospy.sleep((robot_index - 1) * delay_between)

    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # --- control parameters (TUNED FOR SMALLER DETOUR) ---
    angle_tol = 0.15          # [rad] heading tolerance (~9°)
    max_ang_speed = 1.0       # [rad/s]
    v_goal = 1.5              # [m/s] speed toward goal
    v_avoid = 1.0             # [m/s] speed while bypassing
    omega_turn = 0.8          # [rad/s] angular speed when turning in place
    k_ang = 2.0               # heading P gain
    goal_reached_dist = 1.0   # [m] stop when closer than this

    d_block = 3.0             # [m] sonar < d_block => obstacle in front
    d_clear = 3.3             # [m] sonar > d_clear => front considered clear (slightly reduced)
    danger_close = 1.0        # [m] too close during straight bypass (reduced)

    bypass_dist = 2.0         # [m] distance to drive in AVOID_STRAIGHT (reduced from 3.0)
    min_turn_angle = math.radians(25.0)  # [rad] minimum angle in AVOID_TURN (reduced from 45°)

    # --- compute goal position once ---
    rospy.sleep(1.0)

    d0 = float(robot.getDistanceToFlag())
    x0, y0, yaw0 = robot.get_robot_pose()

    goal_x = x0 + d0 * math.cos(yaw0)
    goal_y = y0 + d0 * math.sin(yaw0)

    print(
        f"{robot_name}: computed goal at ({goal_x:.2f}, {goal_y:.2f}) "
        f"from d0={d0:.2f}, pose=({x0:.2f}, {y0:.2f}, yaw={yaw0:.2f})"
    )

    # --- state machine ---
    state = "GO_TO_GOAL"        # or "AVOID_TURN", "AVOID_STRAIGHT"

    avoid_turn_dir = 1.0        # set dynamically
    yaw_turn_start = 0.0

    avoid_straight_start_x = 0.0
    avoid_straight_start_y = 0.0

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        sonar = robot.get_sonar()
        x, y, yaw = robot.get_robot_pose()

        dx = goal_x - x
        dy = goal_y - y
        dist_to_goal = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_heading - yaw)

        # --- goal check ---
        if dist_to_goal < goal_reached_dist:
            robot.set_speed_angle(0.0, 0.0)
            print(f"{robot_name}: goal reached (dist={dist_to_goal:.2f} m). Stopping.")
            rate.sleep()
            continue

        obstacle_in_front = (sonar > 0.0 and sonar < d_block)
        front_clear = (sonar <= 0.0) or (sonar > d_clear)

        vel_cmd = 0.0
        ang_cmd = 0.0

        # ==============================
        #       MAIN STATE MACHINE
        # ==============================
        if state == "GO_TO_GOAL":
            if obstacle_in_front:
                # turn direction AWAY from goal
                avoid_turn_dir = -1.0 if heading_error > 0.0 else 1.0
                yaw_turn_start = yaw
                print(
                    f"{robot_name}: obstacle at {sonar:.2f} m "
                    f"-> AVOID_TURN (dir={avoid_turn_dir:+.0f})"
                )
                state = "AVOID_TURN"
                vel_cmd = 0.0
                ang_cmd = avoid_turn_dir * omega_turn
            else:
                # standard go-to-goal behavior
                if abs(heading_error) > angle_tol:
                    ang_cmd = k_ang * heading_error
                    if ang_cmd > max_ang_speed:
                        ang_cmd = max_ang_speed
                    elif ang_cmd < -max_ang_speed:
                        ang_cmd = -max_ang_speed
                    vel_cmd = 0.0
                else:
                    vel_cmd = v_goal
                    ang_cmd = 0.0

        elif state == "AVOID_TURN":
            vel_cmd = 0.0
            ang_cmd = avoid_turn_dir * omega_turn

            rotated = abs(normalize_angle(yaw - yaw_turn_start))

            # front clear AND we have turned at least the (smaller) minimum angle
            if front_clear and rotated > min_turn_angle:
                print(
                    f"{robot_name}: front clear (sonar={sonar:.2f}), "
                    f"rotated={math.degrees(rotated):.1f}° -> AVOID_STRAIGHT"
                )
                state = "AVOID_STRAIGHT"
                avoid_straight_start_x = x
                avoid_straight_start_y = y
                vel_cmd = v_avoid
                ang_cmd = 0.0

        elif state == "AVOID_STRAIGHT":
            traveled = math.hypot(x - avoid_straight_start_x,
                                   y - avoid_straight_start_y)

            # keep some safety if we see something very close
            if sonar > 0.0 and sonar < danger_close:
                vel_cmd = v_avoid * 0.6
                ang_cmd = avoid_turn_dir * omega_turn
                print(
                    f"{robot_name}: CLOSE during bypass (sonar={sonar:.2f}), "
                    f"turning outward."
                )
            else:
                vel_cmd = v_avoid
                ang_cmd = 0.0

            # new obstacle appears early in bypass -> re-do turn
            if obstacle_in_front and traveled < bypass_dist * 0.5:
                print(
                    f"{robot_name}: new obstacle during bypass (sonar={sonar:.2f}) "
                    f"-> AVOID_TURN"
                )
                state = "AVOID_TURN"
                yaw_turn_start = yaw
                vel_cmd = 0.0
                ang_cmd = avoid_turn_dir * omega_turn

            # done bypassing -> go back to goal
            elif traveled >= bypass_dist:
                print(
                    f"{robot_name}: bypass distance {traveled:.2f} m reached "
                    f"-> GO_TO_GOAL"
                )
                state = "GO_TO_GOAL"

        else:
            print(f"{robot_name}: unknown state '{state}', resetting to GO_TO_GOAL")
            state = "GO_TO_GOAL"
            vel_cmd = 0.0
            ang_cmd = 0.0

        robot.set_speed_angle(vel_cmd, ang_cmd)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous=True)
    run_demo()

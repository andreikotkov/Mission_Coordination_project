#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from evry_project_plugins.srv import DistanceToFlag


class Robot:
    def __init__(self, robot_name):
        """Constructor of the class Robot
        The required publishers / subscribers are created.
        The attributes of the class are initialized

        Args:
            robot_name (str): Name of the robot, like robot_1, robot_2 etc. To be used for your subscriber and publisher with the robot itself
        """
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0  # Sonar distance
        self.x, self.y = 0.0, 0.0  # coordinates of the robot
        self.yaw = 0.0  # yaw angle of the robot
        self.robot_name = robot_name

        '''Listener and publisher'''

        rospy.Subscriber(self.robot_name + "/sensor/sonar_front",
                         Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom",
                         Odometry, self.callbackPose)
        self.cmd_vel_pub = rospy.Publisher(
            self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Range): Message that contains the distance separating the US sensor from a potential obstacle
        """
        self.sonar = msg.range

    def get_sonar(self):
        """Method that returns the distance separating the ultrasonic sensor from a potential obstacle
        """
        return self.sonar

    def callbackPose(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Odometry): Message that contains the coordinates of the agent
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x,
                           quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion_list)
        self.yaw = yaw

    def get_robot_pose(self):
        """Method that returns the position and orientation of the robot"""
        return self.x, self.y, self.yaw

    def constraint(self, val, min=-2.0, max=2.0):
        """Method that limits the linear and angular velocities sent to the robot

        Args:
            val (float): [Desired velocity to send
            min (float, optional): Minimum velocity accepted. Defaults to -2.0.
            max (float, optional): Maximum velocity accepted. Defaults to 2.0.

        Returns:
            float: Limited velocity whose value is within the range [min; max]
        """
        # DO NOT TOUCH
        if val < min:
            return min
        if val > max:
            return max
        return val

    def set_speed_angle(self, linear, angular):
        """Method that publishes the proper linear and angular velocities commands on the related topic to move the robot

        Args:
            linear (float): desired linear velocity
            angular (float): desired angular velocity
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear)
        cmd_vel.angular.z = self.constraint(angular, min=-1, max=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        """Get the distance separating the agent from a flag. The service 'distanceToFlag' is called for this purpose.
        The current position of the robot and its id should be specified. The id of the robot corresponds to the id of the flag it should reach


        Returns:
            float: the distance separating the robot from the flag
        """
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            # int(robot_name[-1]) corresponds to the id of the robot. It is also the id of the related flag
            result = service(pose, int(self.robot_name[-1]))
            return result.distance
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


def run_demo():
    '''
    ███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████
    1st strategy assumes the simplest logic where we do not calculating the position of flags, but only make tries to dicrease the distance 
    to goal;
    if distance gets bigger the robot makes turn and tries again until it reaches the goal.
    In case of obstacle the robot also make turn until the sonar sensor does not detect free space to move -> then it repeats the 
    previous logic.
    ███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████
    '''
    """Main loop: obstacle avoidance with distance-based correction"""

    robot_name = rospy.get_param("~robot_name", "robot_1")
    robot = Robot(robot_name)
    print(f"Robot : {robot_name} is starting with distance-aware avoidance strategy..")

    # --- Sequential start based on robot index ---
    # robot_1 -> index 1 -> delay 0 s
    # robot_2 -> index 2 -> delay 1 s
    # robot_3 -> index 3 -> delay 2 s
    try:
        robot_index = int(robot_name.split('_')[-1])
    except ValueError:
        robot_index = 1  # fallback if name is weird

    delay_between = 0.5   # seconds between robots
    startup_delay = (robot_index - 1) * delay_between
    print(f"{robot_name}: waiting {startup_delay:.1f} s before starting control loop")
    rospy.sleep(startup_delay)

    # PID gains (on distance to flag) used in GO state
    Kp = 0.8
    Ki = 0.0
    Kd = 0.1

    integral = 0.0
    prev_error = 0.0
    prev_time = rospy.get_time()

    flag_reached_dist = 2.0        # [m] stop when closer than this

    safe_obstacle_dist = 4.0       # [m] see obstacle if sonar < this
    clear_obstacle_dist = 4.5      # [m] definitely clear if sonar > this

    # For distance comparison
    prev_distance = None           # last loop distance
       
    eps_dist = 0.05                # [m] tolerance for "improvement"

    # State machine
    state = "GO"                   # "GO", "AVOID_ROTATE"

    # Choose basic turning direction from robot index (optional)
    try:
        robot_index = int(robot_name.split('_')[-1])
    except ValueError:
        robot_index = 1
    base_turn_dir = 1.0 if (robot_index % 2 == 1) else -1.0  # odd robots left, even right

    while not rospy.is_shutdown():

        sonar = robot.get_sonar()
        distance = float(robot.getDistanceToFlag())
        x, y, yaw = robot.get_robot_pose()

        if prev_distance is None:
            prev_distance = distance

        current_time = rospy.get_time()
        dt = current_time - prev_time
        if dt <= 0.0:
            dt = 1e-3

        print(f"[{robot_name}] state={state}, dist={distance:.2f} m, sonar={sonar:.2f} m")

        # ============= FLAG REACHED =============
        if distance < flag_reached_dist:
            velocity = 0.0
            angle = 0.0
            integral = 0.0
            prev_error = distance
            print(f"{robot_name}: Flag reached, stopping.")
            prev_time = current_time
            robot.set_speed_angle(velocity, angle)
            rospy.sleep(0.2)
            prev_distance = distance
            continue

    

        # --------- 1) GO: normal motion ---------
        if state == "GO":
            # If obstacle ahead -> go to avoidance
            if 0.0 < sonar < safe_obstacle_dist:
                state = "AVOID_ROTATE"
                velocity = 0.0
                angle = 0   
                integral = 0.0
                prev_error = distance
                print(f"{robot_name}: Obstacle at {sonar:.2f} m, switching to AVOID_ROTATE.")
                

            # If no obstacle but distance got worse -> try local search
            elif distance > prev_distance + eps_dist:
                print(f"{robot_name}: Distance increased (from {prev_distance:.2f} to {distance:.2f}), starting local search.")
                
                test_start_distance = distance
                velocity = 0.0
                angle = base_turn_dir * 4 
                rospy.sleep(0.5) 

            else:
                # Normal PID go-to-goal on distance
                error = distance
                integral += error * dt
                derivative = (error - prev_error) / dt

                u = Kp * error + Ki * integral + Kd * derivative
                if u < 0.0:
                    u = 0.0
                if u > 5.0:
                    u = 5.0   

                velocity = u
                angle = 0.0
                prev_error = error
                print(f"{robot_name}: GO, PID v={velocity:.2f} m/s")

        # ----- 2) AVOID_ROTATE: see obstacle -----
        elif state == "AVOID_ROTATE":
            # Keep rotating away as long as obstacle is close
            if 0.0 < sonar < clear_obstacle_dist:
                velocity = 0.0
                angle = base_turn_dir * 1
                print(f"{robot_name}: AVOID_ROTATE, turning (sonar={sonar:.2f} m)")
            else:
                # Obstacle no longer visible 
                state = "GO"
                

        else:
            # Failsafe
            velocity = 0.0
            angle = 0.0
            state = "GO"

        # ========================================

        prev_time = current_time
        prev_distance = distance

        robot.set_speed_angle(velocity, angle)
        rospy.sleep(0.2)






if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous=True)
    run_demo()

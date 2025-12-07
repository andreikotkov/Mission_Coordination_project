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
            
            # Extract ID from robot_name (e.g. robot_1 -> 1)
            try:
                robot_id = int(self.robot_name.split('_')[-1])
            except ValueError:
                robot_id = 1
                
            result = service(pose, robot_id)
            return result.distance
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return float("inf")


def run_demo():
    '''
    ███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████
        This strategy uses an *Advanced Artificial Potential Field (APF)* with *Geometric Awareness*. It treats the robot 
        not as a point, but as an *square* with a virtual safety buffer, ensuring that the robot's corners do not clip 
        obstacles during turns.

        Navigation is driven by calculated physics forces:
      - Attraction: Pulls the robot toward the computed goal coordinates.
      - Repulsion: Pushes the robot away from obstacles based on its physical size + safety buffer.

        To handle complex traps (like trees or "blind" corners), it implements a *Tactical Retreat and Scan* logic:
        1) CRITICAL REVERSE: If the robot gets dangerously close (< 0.9m), it immediately stops and reverses in a straight line 
           to gain maneuvering space.
        2) SCAN and SEARCH: It then rotates in place (ignoring the goal) until the sensor detects an open path (> 3.5m).
        3) CLEARANCE: After dodging, it forces a forward drive phase to make robot's body fully past the obstacle.
    ███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████
    '''
    robot_name = rospy.get_param("~robot_name", "robot_1")
    robot = Robot(robot_name)
    print(f"{robot_name}: Starting Strategy 7 (Big Safety Zones)...")

    try:
        idx = int(robot_name.split('_')[-1])
    except:
        idx = 1
    rospy.sleep(idx * 0.5)

    # 1. CALCULATE GOAL
    d0 = float(robot.getDistanceToFlag())
    x0, y0, yaw0 = robot.get_robot_pose()
    goal_x = x0 + d0 * math.cos(yaw0)
    goal_y = y0 + d0 * math.sin(yaw0)
    
    # =========================================================
    # 2. GEOMETRY + SAFETY BUFFER
    # =========================================================
    ROBOT_WIDTH = 1.0
    ROBOT_LENGTH = 1.0
    
    
    SAFETY_BUFFER = 0.4 
    
    # Calculate radius based on bigger size
    # treating robot as 1.4 x 1.4
    CORNER_RADIUS = math.hypot((ROBOT_WIDTH + SAFETY_BUFFER)/2.0, 
                               (ROBOT_LENGTH + SAFETY_BUFFER)/2.0)
    
    FRONT_OFFSET = ROBOT_LENGTH / 2.0
    
    print(f"Safe Corner Radius: {CORNER_RADIUS:.2f}m")

    # --- TUNING ---
    K_att = 2.0         
    K_rep = 25.0        
    
    # DISTANCES (Increased for safety)
    dist_avoid_trigger = 2.5  # Start avoiding at 2.5m
    dist_emergency = 0.9      # Panic if closer than 0.9m
    dist_scan_clear = 3.5     # Need 3.5m of space to feel safe after backing up
    
    # TIMERS (Increased)
    clearance_duration = 2.8  
    reverse_duration = 1.5    
    
    # STATES
    # 0 = SEEK GOAL
    # 1 = APF AVOID
    # 2 = CLEARING (Straight Drive)
    # 3 = REVERSING (Straight Back)
    # 4 = FIND_OPENING (Rotate in place)
    state = 0
    state_start_time = 0.0

    rate = rospy.Rate(10)

    def normalize_angle(angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    while not rospy.is_shutdown():
        x, y, yaw = robot.get_robot_pose()
        sonar = robot.get_sonar()
        
        # Goal Check
        dist_to_goal = math.hypot(goal_x - x, goal_y - y)
        if dist_to_goal < 0.5:
            robot.set_speed_angle(0.0, 0.0)
            print("Goal reached!")
            break

        valid_sonar = sonar if sonar > 0.01 else 999.0
        current_time = rospy.get_time()

        # =========================================================
        # STATE TRANSITIONS
        # =========================================================
        
        # 1. EMERGENCY MONITOR (Global Priority)
        if state != 3 and state != 4 and valid_sonar < dist_emergency:
            state = 3
            state_start_time = current_time
            print(f"CRITICAL ({valid_sonar:.2f}m)! STOP & REVERSE")

        # STATE 0: SEEK GOAL
        elif state == 0:
            if valid_sonar < dist_avoid_trigger:
                state = 1
                print("Obstacle (Buffer Zone) -> Avoid")

        # STATE 1: APF AVOID
        elif state == 1:
            
            if valid_sonar > dist_avoid_trigger + 0.5:
                state = 2
                state_start_time = current_time
                print("Path Clear -> Clearing Phase")

        # STATE 2: CLEARING FORWARD
        elif state == 2:
            if valid_sonar < dist_avoid_trigger:
                state = 1
            elif (current_time - state_start_time) > clearance_duration:
                state = 0
                print("Clearance Done -> Resume Goal")
        
        # STATE 3: REVERSING
        elif state == 3:
            if (current_time - state_start_time) > reverse_duration:
                state = 4 
                print("Reverse Done -> SCANNING")

        # STATE 4: FIND OPENING
        elif state == 4:
            if valid_sonar > dist_scan_clear:
                state = 2 
                state_start_time = current_time
                print(f"FOUND WIDE OPENING ({valid_sonar:.1f}m) -> GO!")


        # =========================================================
        # MOTION CONTROL
        # =========================================================
        linear_cmd = 0.0
        angular_cmd = 0.0

        if state == 0: # SEEK GOAL
            desired_yaw = math.atan2(goal_y - y, goal_x - x)
            err = normalize_angle(desired_yaw - yaw)
            angular_cmd = 2.0 * err
            linear_cmd = 1.5 * math.cos(err)

        elif state == 1: # APF AVOID
            # Geometric Repulsion 
            dist_center = valid_sonar + FRONT_OFFSET
            if dist_center <= CORNER_RADIUS: dist_center = CORNER_RADIUS + 0.01
            strength = K_rep * (1.0/(dist_center - CORNER_RADIUS))
            
            # Stronger side push to clear the bigger buffer
            f_rep_x = -math.cos(yaw) - 3.0 * math.sin(yaw) 
            f_rep_y = -math.sin(yaw) + 3.0 * math.cos(yaw)
            
            target = math.atan2(f_rep_y, f_rep_x)
            err = normalize_angle(target - yaw)
            
            angular_cmd = 3.0 * err
            linear_cmd = 0.4 

        elif state == 2: # CLEARING
            linear_cmd = 1.2
            angular_cmd = 0.0

        elif state == 3: # REVERSING
            linear_cmd = -0.8 
            angular_cmd = 0.0 

        elif state == 4: # FIND OPENING
            linear_cmd = 0.0
            desired_yaw = math.atan2(goal_y - y, goal_x - x)
            err = normalize_angle(desired_yaw - yaw)
            
            # If goal blocked, force spin
            if abs(err) < 0.5: 
                angular_cmd = 1.5
            else:
                angular_cmd = 1.5 if err > 0 else -1.5

        # Safety Clamp
        if state != 3 and linear_cmd < 0.0: 
            linear_cmd = 0.0
        
        robot.set_speed_angle(linear_cmd, angular_cmd)
        rate.sleep()


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous=True)
    run_demo()
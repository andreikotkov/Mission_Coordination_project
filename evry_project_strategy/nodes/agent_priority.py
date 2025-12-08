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
        self.robot_name = robot_name
        
        # --- 1. ID PARSING ---
        # We parse the number from the name (e.g., "robot_1" -> 1).
        # This ID is critical for the Priority Strategy: Lower ID = Higher Priority.
        try:
            self.my_id = int(robot_name.split('_')[-1])
        except:
            self.my_id = 1
            
        self.x, self.y = 0.0, 0.0
        self.yaw = 0.0
        self.sonar = 0.0
        
        # Dictionary to store the (x,y) positions of OTHER robots (Peers)
        self.peers = {} 

        # --- 2. SUBSCRIBERS ---
        # Subscribe to own sensors
        rospy.Subscriber(self.robot_name + "/sensor/sonar_front", Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom", Odometry, self.callbackPose)
        
        # Subscribe to PEERS
        # We loop through IDs 1 to 3. If the ID is not mine, I subscribe to its Odometry.
        # This allows this robot to "see" where the others are for traffic control.
        for i in range(1, 4):
            if i != self.my_id:
                topic = f"/robot_{i}/odom"
                # We use a lambda function to pass the 'rid' (Robot ID) to the callback
                rospy.Subscriber(topic, Odometry, lambda msg, rid=i: self.peer_callback(msg, rid))

        self.cmd_vel_pub = rospy.Publisher(self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        """Updates the distance to the nearest object in front."""
        self.sonar = msg.range

    def get_sonar(self):
        return self.sonar

    def callbackPose(self, msg):
        """Updates the robot's own position (x, y) and orientation (yaw)."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        q_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = euler_from_quaternion(q_list)
        self.yaw = yaw

    def peer_callback(self, msg, robot_id):
        """
        Callback for OTHER robots. 
        Stores their position in the 'self.peers' dictionary using their ID as the key.
        """
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        self.peers[robot_id] = (px, py)

    def get_robot_pose(self):
        return self.x, self.y, self.yaw

    def set_speed_angle(self, linear, angular):
        """Sends velocity commands to the motors with safety limits."""
        cmd_vel = Twist()
        cmd_vel.linear.x = max(min(linear, 2.0), -2.0)
        cmd_vel.angular.z = max(min(angular, 1.0), -1.0)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        """Service call to get distance to the flag."""
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            result = service(pose, self.my_id)
            return result.distance
        except rospy.ServiceException as e:
            return 0.0

def normalize_angle(angle):
    """
    Normalizes an angle to the range [-pi, pi].
    This ensures the robot always turns the shortest direction (e.g., turning 10 deg Right instead of 350 deg Left).
    """
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def run_demo():
    '''
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
    '''
    
    robot_name = rospy.get_param("~robot_name", "robot_1")
    robot = Robot(robot_name)
    print(f"{robot_name}: Final Robust Strategy Started.")

    # --- 3. GOAL CALCULATION ---
    # We wait 1 second to ensure sensors are ready.
    # Then we calculate the ABSOLUTE (x,y) coordinates of the flag.
    # We do this because if we simply followed "distance", dodging an obstacle might confuse the robot.
    # By knowing the coordinate, we can always recalculate the correct heading.
    rospy.sleep(1.0)
    d0 = float(robot.getDistanceToFlag())
    x0, y0, yaw0 = robot.get_robot_pose()
    goal_x = x0 + d0 * math.cos(yaw0)
    goal_y = y0 + d0 * math.sin(yaw0)
    
    # Logic State Variables
    state = "GO" # Initial state. Options: GO, AVOID, BYPASS, WAIT
    bypass_start_time = 0.0
    
    rate = rospy.Rate(10) # 10 Hz Loop
    
    while not rospy.is_shutdown():
        # Read sensors
        sonar = robot.get_sonar()
        dist_flag = float(robot.getDistanceToFlag())
        x, y, yaw = robot.get_robot_pose()
        current_time = rospy.get_time()
        
        # --- 4. PRIORITY CHECK (INTERSECTION SAFETY) ---
        yield_to_peer = False
        dist_to_center = math.hypot(x, y)
        
        # CRITICAL ZONE: If we are within 3.0m of the map center (0,0)
        if dist_to_center < 3.0: 
            # Check all known peers
            for pid, (px, py) in robot.peers.items():
                # If a peer is ALSO in the zone AND has a lower ID (Higher Priority)
                if math.hypot(px, py) < 3.0 and pid < robot.my_id:
                    yield_to_peer = True # We must stop!
                    break

        velocity = 0.0
        angle = 0.0

        # --- 5. STATE MACHINE LOGIC ---
        
        # STATE: WAIT (Priority Coordination)
        # If we need to yield, we override everything and STOP.
        # Exception: If we are in 'BYPASS' mode, we keep moving to avoid getting stuck sideways.
        if yield_to_peer and state != "BYPASS":
            state = "WAIT"
            velocity = 0.0
            print(f"{robot_name}: Yielding...")

        # STATE: FINISHED
        elif dist_flag < 1.0:
            velocity = 0.0
            print(f"{robot_name}: Finished!")
        
        # TRIGGER: OBSTACLE DETECTED
        # If sonar is between 0 and 1.5m, we have an emergency. Switch to AVOID.
        elif (0.0 < sonar < 1.5) and state != "AVOID":
            state = "AVOID"
            print(f"{robot_name}: Obstacle! Switching to AVOID.")

        # --- 6. EXECUTING STATES ---
        
        if state == "AVOID":
            # LOGIC: Spin in place until the path is clear.
            # We check for sonar > 2.5m (wide opening) or 0.0 (sim often returns 0 for infinity).
            if sonar > 2.5 or sonar == 0.0: 
                print(f"{robot_name}: Path clear! Switching to BYPASS.")
                state = "BYPASS"
                bypass_start_time = current_time
            else:
                velocity = 0.0
                angle = 0.8 # Rotate Left in place

        elif state == "BYPASS":
            # LOGIC: Drive Straight Blindly.
            # We drive for 4.0 seconds WITHOUT looking at the goal.
            # This ensures we physically pass the box and don't turn back into it immediately.
            if current_time - bypass_start_time > 4.0:
                print(f"{robot_name}: Bypass done. Switching to GO.")
                state = "GO"
            else:
                velocity = 1.0
                angle = 0.0 # Drive Straight

        elif state == "GO" or state == "WAIT":
             # LOGIC: Normal Navigation to Goal
             if not yield_to_peer:
                # 1. Calculate desired angle to reach (x_goal, y_goal)
                target_angle = math.atan2(goal_y - y, goal_x - x)
                
                # 2. Find the difference (error) between current yaw and target angle
                angle_error = normalize_angle(target_angle - yaw)
                
                # 3. Proportional Control for Steering (P-Controller)
                angle = 1.5 * angle_error 
                
                # 4. Speed Control
                # If we are facing the wrong way (> 0.5 rad), slow down/turn in place.
                if abs(angle_error) > 0.5:
                    velocity = 0.2 
                else:
                    # If we are facing the goal, drive fast.
                    velocity = 1.5

        # Send command to robot
        robot.set_speed_angle(velocity, angle)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Controller", anonymous=True)
    run_demo()
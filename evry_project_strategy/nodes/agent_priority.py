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
        try:
            self.my_id = int(robot_name.split('_')[-1])
        except:
            self.my_id = 1
        self.x, self.y = 0.0, 0.0
        self.yaw = 0.0
        self.sonar = 0.0
        self.peers = {}

        rospy.Subscriber(self.robot_name + "/sensor/sonar_front", Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom", Odometry, self.callbackPose)
        
        # Subscribe to peers for priority
        for i in range(1, 4):
            if i != self.my_id:
                topic = f"/robot_{i}/odom"
                rospy.Subscriber(topic, Odometry, lambda msg, rid=i: self.peer_callback(msg, rid))

        self.cmd_vel_pub = rospy.Publisher(self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        self.sonar = msg.range

    def get_sonar(self):
        return self.sonar

    def callbackPose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        q_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = euler_from_quaternion(q_list)
        self.yaw = yaw

    def peer_callback(self, msg, robot_id):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        self.peers[robot_id] = (px, py)

    def get_robot_pose(self):
        return self.x, self.y, self.yaw

    def set_speed_angle(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = max(min(linear, 2.0), -2.0)
        cmd_vel.angular.z = max(min(angular, 1.0), -1.0)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
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
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def run_demo():
    robot_name = rospy.get_param("~robot_name", "robot_1")
    robot = Robot(robot_name)
    print(f"{robot_name}: Final Robust Strategy Started.")

    # 1. CALCULATE GOAL POSITION
    rospy.sleep(1.0)
    d0 = float(robot.getDistanceToFlag())
    x0, y0, yaw0 = robot.get_robot_pose()
    goal_x = x0 + d0 * math.cos(yaw0)
    goal_y = y0 + d0 * math.sin(yaw0)
    
    # Logic State Variables
    state = "GO" # States: GO, AVOID, BYPASS, WAIT
    bypass_start_time = 0.0
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        sonar = robot.get_sonar()
        dist_flag = float(robot.getDistanceToFlag())
        x, y, yaw = robot.get_robot_pose()
        current_time = rospy.get_time()
        
        # --- PRIORITY CHECK ---
        yield_to_peer = False
        dist_to_center = math.hypot(x, y)
        if dist_to_center < 3.0: 
            for pid, (px, py) in robot.peers.items():
                if math.hypot(px, py) < 3.0 and pid < robot.my_id:
                    yield_to_peer = True
                    break

        velocity = 0.0
        angle = 0.0

        # --- STATE MACHINE ---
        
        # 1. PRIORITY WAIT (Overrides everything else unless we are bypassing)
        if yield_to_peer and state != "BYPASS":
            state = "WAIT"
            velocity = 0.0
            print(f"{robot_name}: Yielding...")

        # 2. GOAL REACHED
        elif dist_flag < 1.0:
            velocity = 0.0
            print(f"{robot_name}: Finished!")
        
        # 3. OBSTACLE DETECTED -> SWITCH TO AVOID
        elif (0.0 < sonar < 1.5) and state != "AVOID":
            state = "AVOID"
            print(f"{robot_name}: Obstacle! Switching to AVOID.")

        # --- HANDLING STATES ---
        
        if state == "AVOID":
            # Spin Left until path is clear
            if sonar > 2.5 or sonar == 0.0: # Clear path found (sonar=0 usually means inf/clear in sim)
                print(f"{robot_name}: Path clear! Switching to BYPASS.")
                state = "BYPASS"
                bypass_start_time = current_time
            else:
                velocity = 0.0
                angle = 0.8 # Rotate Left

        elif state == "BYPASS":
            # Drive Straight for 4 seconds to clear the side of the box
            if current_time - bypass_start_time > 4.0:
                print(f"{robot_name}: Bypass done. Switching to GO.")
                state = "GO"
            else:
                velocity = 1.0
                angle = 0.0 # Drive Straight (Don't look at goal yet!)

        elif state == "GO" or state == "WAIT":
             if not yield_to_peer:
                # Normal Navigation to Flag
                target_angle = math.atan2(goal_y - y, goal_x - x)
                angle_error = normalize_angle(target_angle - yaw)
                
                # Proportional Steering
                angle = 1.5 * angle_error 
                
                # Speed control based on facing
                if abs(angle_error) > 0.5:
                    velocity = 0.2 
                else:
                    velocity = 1.5

        robot.set_speed_angle(velocity, angle)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Controller", anonymous=True)
    run_demo()
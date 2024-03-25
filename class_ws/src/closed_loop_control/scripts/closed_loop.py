#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

# Puzzlebot class to calculate current state with basic kinematics and an open-loop routine

class Puzzlebot:
    def __init__(self):
        # Initialise the node 
        rospy.init_node("puzzlebot_open_loop")
        self.loop_rate = rospy.Rate(400)
        self.sample_time = 0.005

        # Robot description
        self.wheel_radius = 0.05
        self.wheel_base = 0.19

        # Variables/Parameters to be used
        self.target_path = [
            [3.1,  0  ],
            [3.9,  3.1],
            [0.8,  4.4],
            [-0.2, 0.7]
        ]

        # Closed Loop

        # Kinematics
        self.right_wheel = 0.0
        self.left_wheel = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.distance = 0.0
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.last_time = 0.0

        self.current_iteration = 0

        # PID Parameters
        self.rotation_pid = {
            "kp": 0.9,
            "ki": 0.0,
            "kd": 0.0,
            "integral": 0.0,
            "last_error": 0.0
        }

        self.translation_pid = {
            "kp": 0.08,
            "ki": 0.00001,
            "kd": 0.0,
            "integral": 0.0,
            "last_error": 0.0
        }

        # Setup Publishers and Subscribers
        self.pub_cmd_vel = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)
        self.sub_right_wheel = rospy.Subscriber('/puzzlebot_1/wr', Float32, self.right_wheel_callback)
        self.sub_left_wheel = rospy.Subscriber('/puzzlebot_1/wl', Float32, self.left_wheel_callback)

    # Right wheel callback in rad/s
    def right_wheel_callback(self, msg):
        self.right_wheel = msg.data

    # Left wheel callback in rad/s
    def left_wheel_callback(self, msg):
        self.left_wheel = msg.data

    # CMD_VEL publisher
    def set_twist(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.pub_cmd_vel.publish(cmd_vel)
        #rospy.loginfo(f"Linear: {linear}, Angular: {angular}")

    def rotation_control(self, error, dt):
        self.rotation_pid["integral"] += error * dt
        derivative = (error - self.rotation_pid["last_error"]) / dt
        self.rotation_pid["last_error"] = error
        return self.rotation_pid["kp"] * error + self.rotation_pid["ki"] * self.rotation_pid["integral"] - self.rotation_pid["kd"] * derivative
    
    def translation_control(self, error, dt):
        self.translation_pid["integral"] += error
        derivative = error - self.translation_pid["last_error"]
        self.translation_pid["last_error"] = error
        return self.translation_pid["kp"] * error + self.translation_pid["ki"] * self.translation_pid["integral"] - self.translation_pid["kd"] * derivative

    def stop(self):
        self.set_twist(0.0, 0.0)


    def run(self):
        # Linear and angular velocity calculation with basic kinematics
        self.loop_rate.sleep()

        dt = rospy.Time.now().to_sec() - self.last_time
        if dt < self.sample_time:
            return
        
        self.x += self.wheel_radius * (self.right_wheel + self.left_wheel)/2 * math.cos(self.angle) * dt
        self.y += self.wheel_radius * (self.right_wheel + self.left_wheel)/2 * math.sin(self.angle) * dt
        self.angle += self.wheel_radius * (self.right_wheel - self.left_wheel) / self.wheel_base * dt

        #rospy.loginfo(f"X: {self.x}, Y: {self.y}, T: {self.angle}")
        # print with 2 decimal places
        rospy.loginfo(f"X: {self.x:.2f}, Y: {self.y:.2f}, T: {self.angle:.2f}")

        required_angle = math.atan2(self.target_path[self.current_iteration][1] - self.y, self.target_path[self.current_iteration][0] - self.x)
        if required_angle < -.1:
            required_angle += 2*math.pi
        self.error_angle = required_angle - self.angle
        self.error_distance = math.sqrt((self.target_path[self.current_iteration][0] - self.x)**2 + (self.target_path[self.current_iteration][1] - self.y)**2)

        if abs(self.error_angle) < 0.15 and abs(self.error_distance) < 0.25:
            self.stop()
            rospy.loginfo("Reached target")
            rospy.sleep(3.0)
            self.current_iteration += 1
            #self.x = self.target_path[self.current_iteration][0]
            self.translation_pid["integral"] = 0.0
            self.rotation_pid["integral"] = 0.0
        
        v = self.translation_control(self.error_distance, dt)
        w = self.rotation_control(self.error_angle, dt)

        if abs(self.error_angle) > 0.8:
            v = 0.0

        self.set_twist(v, w)

        #rospy.loginfo(f"V: {v}, W: {w}")
        rospy.loginfo(f"EA: {self.error_angle:.2f}, ED: {self.error_distance:.2f}")

        self.last_time = rospy.Time.now().to_sec()

        if self.current_iteration >= len(self.target_path):
            self.stop()
            rospy.loginfo("Path completed")
            rospy.signal_shutdown("Path completed")
            return



if __name__ == "__main__":
    puzzlebot = Puzzlebot()
    try:
        while not rospy.is_shutdown():
            puzzlebot.run()
    except rospy.ROSInterruptException:
        pass
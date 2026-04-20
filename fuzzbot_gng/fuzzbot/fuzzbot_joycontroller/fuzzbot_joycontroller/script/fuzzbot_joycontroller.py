# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import math
import copy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Pro Controller Key Map
B = 0
A = 1
X = 2
Y = 3
SCREEN_SHOT = 5
L = 4
R = 5
ZL = 7
ZR = 8
MINUS= 9
PLUS = 10
HOME = 11
L3 = 12
R3 = 13

LX = 0
LY = 1
RX = 2
RY = 3
CROSS_X = 4
CROSS_Y = 5

# GPD
LX = 0
LY = 1
RX = 3
RY = 4

class ArmController(Node):

    def __init__(self):
        super().__init__('fuzzbot_joycontroller')
        #Params
        self.declare_parameter("linear_v", 0.66)
        self.declare_parameter("linear_a", 2.0)
        self.declare_parameter("anglar_w", 6.660176426)
        self.declare_parameter("pan_limit", math.pi / 5.)
        self.declare_parameter("tilt_limit", 0.5)
        self.declare_parameter("joint_w", 3.14)
        self.declare_parameter("arm_goal_time_normal", 0.2)
        self.declare_parameter("arm_goal_time_home", 0.8)
    
        self.linear_v_ = self.get_parameter("linear_v").get_parameter_value().double_value
        self.linear_a_ = self.get_parameter("linear_a").get_parameter_value().double_value
        self.anglar_w_ = self.get_parameter("anglar_w").get_parameter_value().double_value
        self.pan_limit_ = self.get_parameter("pan_limit").get_parameter_value().double_value
        self.tilt_limit_ = self.get_parameter("tilt_limit").get_parameter_value().double_value
        self.joint_w_ = self.get_parameter("joint_w").get_parameter_value().double_value
        self.arm_goal_time_normal_ = self.get_parameter("arm_goal_time_normal").get_parameter_value().double_value
        self.arm_goal_time_home_ = self.get_parameter("arm_goal_time_home").get_parameter_value().double_value
    
        TIMER_PERIOD_SEC = 0.05
        ROS_QUEUE_SIZE = 10
        
        self.joy_ = None
        self.home_point_ = [0., -1.1, 1.87, 0., .98, 0.] # (joint angle ✕ 6)
        self.initial_point_ = [0., 0., 0., 0., 0., 0.]
        self.joint_point_ = self.home_point_[:]

        self.joint_w_ *= TIMER_PERIOD_SEC

        self.prev_v_ = 0.
        self.linear_a_ *= TIMER_PERIOD_SEC
        
        self.joint_msg_ = JointTrajectory()
        self.joint_msg_.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = self.joint_point_[:]
        self.joint_msg_.points.append(point)
        

        # Publisher
        self.twist_pub_ = self.create_publisher(Twist, 'cmd_vel', ROS_QUEUE_SIZE)
        self.pantilt_pub_ = self.create_publisher(Float64MultiArray, 'pantilt_controller/commands', ROS_QUEUE_SIZE)
        self.joint_pub_ = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', ROS_QUEUE_SIZE)
        
        # Subscriber
        self.joy_sub_ = self.create_subscription(Joy, 'joy', self.joy_cb, ROS_QUEUE_SIZE)
        
        # Action Client
        self.gripper_client_ = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
        
        self.timer_ = self.create_timer(TIMER_PERIOD_SEC, self.timer_cb) #20Hz

    def timer_cb(self):
        if self.joy_ == None:
            return
        # Wheel
        twist = Twist()
        v = self.linear_v_ * self.joy_.axes[LY]
        if (v - self.prev_v_) > self.linear_a_:
            v = self.prev_v_ + self.linear_a_
        elif -(v - self.prev_v_) > self.linear_a_:
            v = self.prev_v_ - self.linear_a_
        self.prev_v_ = v

        twist.linear.x = v
        if self.joy_.buttons[L]:
            twist.angular.z = self.anglar_w_
        elif self.joy_.buttons[R]:
            twist.angular.z = -self.anglar_w_
        else:
            twist.angular.z = 0.
        self.twist_pub_.publish(twist)
        
        # Pantilt
        pantilt_msg = Float64MultiArray()
        pantilt_msg.data = [self.pan_limit_ * self.joy_.axes[RX], self.tilt_limit_ * self.joy_.axes[RY]]
        self.pantilt_pub_.publish(pantilt_msg)
    
    def joy_cb(self, msg):
        self.joy_ = copy.copy(msg)
        
    def send_goal(self, position):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = -1.0
        #self.gripper_client_.wait_for_server()
        return self.gripper_client_.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
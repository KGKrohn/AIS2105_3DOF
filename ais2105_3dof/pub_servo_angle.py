import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ServoAnglePlublish(Node):
    def __init__(self):
        super().__init__('Pub_joint')
        #Initialize pub
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 0)
        self.joint_trajectory_msg = JointTrajectory()
        self.joint_trajectory_msg.joint_names = ['ard_motorA', 'ard_motorB', 'ard_motorC']
        #Initialize sub
        
        self.subscription_A = self.create_subscription(Float64, 'servo_angle_A', self.listener_callback_A, 10)
        self.subscription_B = self.create_subscription(Float64, 'servo_angle_B', self.listener_callback_B, 10)
        self.subscription_C = self.create_subscription(Float64, 'servo_angle_C', self.listener_callback_C, 10)

        self.message_A = 10.0
        self.message_B = 10.0
        self.message_C = 10.0

        timer_period = 0.1  # dt
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        joint_trajectory_point = JointTrajectoryPoint()
        print("sub",self.message_A," ",self.message_B ," ", self.message_C)

        # Write servo angle A B and C as a JTPos
        joint_trajectory_point.positions= [self.message_A,
                                           self.message_B,
                                           self.message_C] 
    
        # Write JTPos as JTPoint
        self.joint_trajectory_msg.points = [joint_trajectory_point]

        # Publish
        self.publisher_.publish(self.joint_trajectory_msg)
    
    def listener_callback_A(self, msg):
        self.message_A = msg
    def listener_callback_B(self, msg):
        self.message_B = msg
    def listener_callback_C(self, msg):
        self.message_C = msg

def main(args=None):
    rclpy.init(args=None)
    ServoAnglePlublish_ = ServoAnglePlublish()

    rclpy.spin(ServoAnglePlublish_)
    ServoAnglePlublish_.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
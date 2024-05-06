import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
import time



class ComputeNode(Node):
    def __init__(self):
        super().__init__('ComputeNode')

        #Initialize pub
        self.publisher_A = self.create_publisher(Float64, 'servo_angle_A', 10)
        self.publisher_B = self.create_publisher(Float64, 'servo_angle_B', 10)
        self.publisher_C = self.create_publisher(Float64, 'servo_angle_C', 10)
        self.servo_A = 80
        self.servo_B = 80
        self.servo_C = 80
        

        #Initialize sub
        #self.subscription_x = self.create_subscription(float, 'Ballpos_x', self.listener_callback, 10)
        #self.subscription_y = self.create_subscription(float, 'Ballpos_y', self.listener_callback, 10)
        #self.subscription_z = self.create_subscription(float, 'Ballpos_z', self.listener_callback, 10)
        timer_period = 0.1  # dt
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg_A = Float64()
        msg_B = Float64()
        msg_C = Float64()
        msg_A.data = float(self.servo_A)
        msg_B.data = float(self.servo_B)
        msg_C.data = float(self.servo_C)
        
        self.publisher_A.publish(msg_A)
        self.publisher_B.publish(msg_B)
        self.publisher_C.publish(msg_C)


    def listener_callback(self, msg):
        self.get_logger().info('Servo motor: "%f"' % msg)

    def update_servo_angle(self, servoA, servoB, servoC):
        self.msg_A = float(servoA)
        self.msg_B = float(servoB)
        self.msg_C = float(servoC)


def main ():
    rclpy.init(args=None)
    
    ComputeNode_ = ComputeNode()
    rclpy.spin(ComputeNode_)
    
 
if __name__ == '__main__':
    main()

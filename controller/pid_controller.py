import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class pidControllerNode(Node):
    def __init__(self):
        super().__init__('PID_Node')
        #Initialize pub
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            'PID_PR', 
            10)
        #Initialize sub
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'Ball_pos',
            self.sub_callback,
            10)
        self.servo_A = 10.0
        self.servo_B = 10.0
        self.servo_C = 10.0

        
    def sub_callback(self, msg):

        self.servo_A, self.servo_B, self.servo_C = msg.data

     
        #self.get_logger().info('yepp %f' % self.servo_A)
        servo_angles = self.servo_A, self.servo_B, self.servo_C
        servo_msg = Float32MultiArray(data=servo_angles)
        self.publisher_.publish(servo_msg)
    

def main(args=None):
    rclpy.init(args=None)
    pidControllerNode_ = pidControllerNode()

    rclpy.spin(pidControllerNode_)
    pidControllerNode_.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
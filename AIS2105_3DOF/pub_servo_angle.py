import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64

class ServoAnglePlublish(Node):

    def __init__(self):
        super().__init__('ServoAnglePlublish')
        self.publisher_A = self.create_publisher(Float64, 'MotorA', 0)
        self.publisher_B = self.create_publisher(Float64, 'MotorB', 0)
        self.publisher_C = self.create_publisher(Float64, 'MotorC', 0)
        self.ServoAngle_A = float(0)
        self.ServoAngle_A = float(0)
        self.ServoAngle_A = float(0)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        

    def timer_callback(self):
        msg_A = Float64()
        msg_B = Float64()
        msg_C = Float64()
        msg_A.data = float(self.ServoAngle_A)
        msg_B.data = float(self.ServoAngle_B)
        msg_C.data = float(self.ServoAngle_C)
        self.publisher_A.publish(msg_A)
        self.publisher_A.publish(msg_B)
        self.publisher_A.publish(msg_C)
        #self.get_logger().info(msg_A.data)
        #self.get_logger().info(msg_B.data)
        #self.get_logger().info(msg_C.data)

    def write_serov_angles(self, MotorA, MotorB, MotorC):
        self.ServoAngle_A = MotorA
        self.ServoAngle_B = MotorB
        self.ServoAngle_C = MotorC





def main(args=None):
    rclpy.init(args=args)

    ServoAnglePlublish_ = ServoAnglePlublish()
    ServoAnglePlublish_.ServoAngle_A = 31
    ServoAnglePlublish_.ServoAngle_B = 32
    ServoAnglePlublish_.ServoAngle_C = 33


    rclpy.spin(ServoAnglePlublish_)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ServoAnglePlublish.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
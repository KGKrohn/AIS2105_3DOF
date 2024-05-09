import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ServoAnglePlublish(Node):
    def __init__(self):
        super().__init__('Pub_joint')
        #Initialize pub
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 0)
        self.joint_trajectory_msg = JointTrajectory()
        self.joint_trajectory_msg.joint_names = ['ard_motorA', 'ard_motorB', 'ard_motorC']
        #Initialize sub
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'servo_angles',
            self.servo_angle_callback,
            10)
        self.servo_A = 10.0
        self.servo_B = 10.0
        self.servo_C = 10.0

        
    def servo_angle_callback(self, msg):
        joint_trajectory_point = JointTrajectoryPoint()
        self.servo_A,self.servo_B, self.servo_C = msg.data



        # Write servo angle A B and C as a JTPos
        joint_trajectory_point.positions= [self.servo_A+70,
                                           self.servo_B+70,
                                           self.servo_C+70]
    
        
        self.get_logger().info('pub_servo %f' % self.servo_A)
        self.get_logger().info('pub_servo %f' % self.servo_B)
        self.get_logger().info('pub_servo %f' % self.servo_C)
    
        # Write JTPos as JTPoint
        self.joint_trajectory_msg.points = [joint_trajectory_point]

        # Publish
        self.publisher_.publish(self.joint_trajectory_msg)
    

def main(args=None):
    rclpy.init(args=None)
    ServoAnglePlublish_ = ServoAnglePlublish()

    rclpy.spin(ServoAnglePlublish_)
    ServoAnglePlublish_.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
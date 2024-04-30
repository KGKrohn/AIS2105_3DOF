import rclpy
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
        self.msg_A = float(40.0)
        self.msg_B = float(40.0)
        self.msg_C = float(40.0)
        
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 0)
        self.joint_trajectory_msg = JointTrajectory()
        self.joint_trajectory_msg.joint_names = ['ard_motorA', 'ard_motorB', 'ard_motorC']
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        joint_trajectory_point = JointTrajectoryPoint()

        joint_trajectory_point.positions= [self.msg_A, self.msg_B, self.msg_C]
    
        self.joint_trajectory_msg.points = [joint_trajectory_point]
        self.msg_A += 1.0
        self.msg_B += 1.0
        self.msg_C += 1.0
        if self.msg_A > 130.0:
            self.msg_A = 40.0
            self.msg_B = 40.0
            self.msg_C = 40.0

        self.publisher_.publish(self.joint_trajectory_msg)
        

    def update_servo_angle(self, servoA, servoB, servoC):
        self.msg_A = float(servoA)
        self.msg_B = float(servoB)
        self.msg_C = float(servoC)



def main(args=None):

    rclpy.init(args=None)
    ServoAnglePlublish_ = ServoAnglePlublish()

    #ServoAnglePlublish_.update_servo_angle(servoA,servoB,servoC)

    rclpy.spin(ServoAnglePlublish_)
    ServoAnglePlublish_.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
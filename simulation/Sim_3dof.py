import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import simulering_3dof as sim_3dof
import numpy as np
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from vpython import sphere, vector, color, rate, canvas, cylinder, cos, sin, radians, slider, wtext

class simulationNode(Node):
    def __init__(self):
        super().__init__('sim_Node')
        #Initialize pub
        self.publisher_ball_pos = self.create_publisher(
            Float32MultiArray, 
            'sim_ball_pos', 
            10)
        self.publisher_pitch_roll = self.create_publisher(
            Float32MultiArray, 
            'sim_pitch_roll', 
            10)

        timer_period = 0.01  # dt
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        
    def timer_callback(self):
        self.pos_x, self.pos_y, self.pos_z, self.angle_x, self.angle_y = sim_3dof.get_simulation_status()
        
        ball_pos = self.pos_x, self.pos_y, self.pos_z
        pitch_roll = self.angle_x, self.angle_y
        self.get_logger().info('Servo angle 1: "%f"' % self.angle_x)
        self.get_logger().info('Servo angle 2: "%f"' % self.angle_y)
        
        ball_pos_msg = Float32MultiArray(data=ball_pos)
        pitch_roll_msg = Float32MultiArray(data=pitch_roll)
        self.publisher_ball_pos.publish(ball_pos_msg)
        self.publisher_pitch_roll.publish(pitch_roll_msg)


def main(args=None):
    rclpy.init(args=None)
    simulationNode_ = simulationNode()

    rclpy.spin(simulationNode_)
    simulationNode_.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
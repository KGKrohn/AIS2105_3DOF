import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import numpy as np
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from vpython import sphere, vector, color, rate, canvas, cylinder, cos, sin, radians, slider, wtext

class simulationNode(Node):
    def __init__(self):
        super().__init__('sim_Node')
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.angle_x = 0.0
        self.angle_y = 0.0
        # Constants
        self.g = 9.81  # Acceleration due to gravity (m/s^2)
        self.floor_radius = 10  # Radius of the floor circle
        self.kp, self.ki, self.kd = 0.4, 0.02, 0.8  # PID coefficients
        self.error_sum_x, self.error_sum_y, self.last_error_x, self.last_error_y = 0, 0, 0, 0  # PID terms initialization
        self.max_tilt = 10  # Maximum tilt angle in degrees
        self.tilt_threshold = 0.01  # Threshold for considering tilt to be effectively zero

        # Initialize the scene
        self.scene = canvas(title='3D Ball Rolling Simulation', width=800, height=600, align='left')
        self.scene.camera.pos = vector(0, -20, 30)
        self.scene.camera.axis = vector(0, 20, -30)


        # Floor setup
        self.floor = cylinder(pos=vector(0, 0, 0), axis=vector(0, 0, -0.1), radius=self.floor_radius, color=color.green)

        # Ball setup
        self.ball = sphere(pos=vector(0, 0, 1), radius=0.5, color=color.blue)
        self.ball.velocity = vector(0, 0, 0)  # Initial velocity


        # Target position initially set to (5, 5)
        self.target_position = vector(5, 5, 1)

        # Controls for target position
        self.x_slider = slider(min=-10, max=10, value=5, step=0.1, left=10, bind=lambda: self.update_target_position('x'))
        self.x_title = wtext(text='Target X: 5.0\n')

        self.y_slider = slider(min=-10, max=10, value=5, step=0.1, left=10, bind=lambda: self.update_target_position('y'))
        self.y_title = wtext(text='Target Y: 5.0\n')


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
        
        ball_pos = self.pos_x, self.pos_y, self.pos_z
        pitch_roll = self.angle_x, self.angle_y
        self.update_simulation(0.01)

        
        ball_pos_msg = Float32MultiArray(data=ball_pos)
        pitch_roll_msg = Float32MultiArray(data=pitch_roll)
        self.publisher_ball_pos.publish(ball_pos_msg)
        self.publisher_pitch_roll.publish(pitch_roll_msg)

    def pid_controller(self, target, current, error_sum, last_error, kp, ki, kd, dt):
        error = target - current
        error_sum += error * dt
        delta_error = (error - last_error) / dt
        p = kp * error
        i = ki * error_sum
        d = kd * delta_error
        output = p + i + d
        return output, error, error_sum, error  # returning current error as last_error for next iteration
    
    def update_simulation(self,dt):
        # PID control for X-axis
        output_x, error_x, error_sum_x, new_last_error_x = self.pid_controller(self.target_position.x, self.ball.pos.x, error_sum_x, last_error_x, self.kp, self.ki, self.kd, dt)
        self.last_error_x = new_last_error_x
        tilt_x = self.clamp(output_x, -self.max_tilt, self.max_tilt, self.tilt_threshold)

        # PID control for Y-axis
        output_y, error_y, error_sum_y, new_last_error_y = self.pid_controller(self.target_position.y, self.ball.pos.y, error_sum_y, last_error_y, self.kp,self.ki, self.kd, dt)
        self.last_error_y = new_last_error_y
        tilt_y = self.clamp(output_y, -self.max_tilt, self.max_tilt, self.tilt_threshold)

        # Update floor tilt based on PID outputs
        tilt_x_rad = radians(tilt_x)*10
        tilt_y_rad = radians(tilt_y)*10
        new_axis = vector(sin(tilt_x_rad), sin(tilt_y_rad), cos(tilt_x_rad)*cos(tilt_y_rad))
        self.floor.axis = new_axis.norm() * 0.1

        # Correct ball movement relative to floor tilt
        gravity_component_x = self.g * sin(tilt_x_rad) if abs(tilt_x) > self.tilt_threshold else 0
        gravity_component_y = self.g * sin(tilt_y_rad) if abs(tilt_y) > self.tilt_threshold else 0
        self.ball.velocity.x += gravity_component_x * dt
        self.ball.velocity.y += gravity_component_y * dt
        self.ball.pos.x += self.ball.velocity.x * dt
        self.ball.pos.y += self.ball.velocity.y * dt
        self.adjust_ball_z_position()

    def clamp(self, value, min_value, max_value, threshold):
        clamped_value = max(min_value, min(value, max_value))
        return 0 if abs(clamped_value) < threshold else clamped_value
    
    def pid_controller(self, target, current, error_sum, last_error, kp, ki, kd, dt):
        error = target - current
        error_sum += error * dt
        delta_error = (error - last_error) / dt
        p = kp * error
        i = ki * error_sum
        d = kd * delta_error
        output = p + i + d
        return output, error, error_sum, error  # returning current error as last_error for next iteration
    
def main(args=None):
    rclpy.init(args=None)
    simulationNode_ = simulationNode()

    rclpy.spin(simulationNode_)
    simulationNode_.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
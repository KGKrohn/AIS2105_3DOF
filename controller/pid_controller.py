import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import time
import numpy as np
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

## PID code inspired by https://github.com/KGKrohn/3-Dof_Regtek 
class pidControllerNode(Node):
    def __init__(self):
        super().__init__('PID_Node')
        self.declare_parameter('kp', 0.45)
        self.declare_parameter('ki', 0.14)
        self.declare_parameter('kd', 0.18)
        self.declare_parameter('setpoint', 0.0)
        self.Kp = self.get_parameter('kp').value
        self.Ki = self.get_parameter('ki').value
        self.Kd = self.get_parameter('kd').value
        self.setpoint_x = 0
        self.setpoint_y = 0
        self.lastError = 0
        self.lastError_2 = 0 
        self.IntegralError = 0
        self.IntegralError_2 = 0
        self.DerivativeError = 0
        self.DerivativeError_2 = 0
        self.start_time = 0
        self.x_pos = 0.0
        self.y_pos = 0.0

        #Initialize pub
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            'PID_PR', 
            10)

        self.publisher_JS = self.create_publisher(
            JointState, 
            'joint_states', 
            5)
        
        #Initialize sub
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ball_dist_cm',
            self.sub_callback,
            10)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.joint_state = JointState()
        
        
        
    def sub_callback(self, msg):
        self.x_pos, self.y_pos = msg.data

        self.joint_state.name =['pitch', 'roll', 'ballx', 'bally']
        self.joint_state.position=[0.90, 0.3, 0.10, 0.10]
        
        # Circle and aight figure setpoint.
        #self.setpoint_x =float(80 * np.cos(time.time()) / 10)
        #self.setpoint_y =float(80 * np.sin(time.time()) / 10)
        #self.setpoint_x = float(80 * np.sin(2*time.time())/10)
        #self.setpoint_y = float(80 * np.sin(time.time())/10)

        pid_pitch = self.compute_pitch(self.x_pos)
        pid_roll = self.compute_roll(self.y_pos)

        pid_output = pid_pitch, pid_roll
        pub_msg = Float32MultiArray(data=pid_output)
        self.publisher_.publish(pub_msg)
        self.publisher_JS.publish(self.joint_state)


    def compute_pitch(self, systemValue): #source https://github.com/KGKrohn/3-Dof_Regtek
        dt = 0.03
        error = self.setpoint_y - systemValue
        self.IntegralError += error * dt
        self.IntegralError = np.clip(self.IntegralError, a_min=-5, a_max=5)  # integral windup
        self.DerivativeError = (error - self.lastError) / dt
        
        output = (-self.Kp * error) + (-self.Ki * self.IntegralError) + (-self.Kd * self.DerivativeError)
    
        self.lastError = error
        return output


    def compute_roll(self, systemValue): #source https://github.com/KGKrohn/3-Dof_Regtek
        dt = 0.03
        error = self.setpoint_x - systemValue
        self.IntegralError_2 += error * dt
        self.IntegralError_2 = np.clip(self.IntegralError_2, a_min=-5, a_max=5)  # integral windup
        self.DerivativeError_2 = (error - self.lastError_2) / dt

        output = (-self.Kp * error) + (-self.Ki * self.IntegralError_2) + (-self.Kd * self.DerivativeError_2)

        self.lastError_2 = error

        return output
    
    def parameter_callback(self, params:list[Parameter]):
        for param in params:
            if param.name == 'kp':
                self.Kp = param.value
            if param.name == 'ki':
                self.Ki = param.value
            if param.name == 'kd':
                self.Kd = param.value

    
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=None)
    pidControllerNode_ = pidControllerNode()

    rclpy.spin(pidControllerNode_)
    pidControllerNode_.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
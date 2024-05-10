import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import numpy as np
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class pidControllerNode(Node):
    def __init__(self):
        super().__init__('PID_Node')
        self.declare_parameter('kp',0.39)
        self.declare_parameter('ki',0.62)
        self.declare_parameter('kd',0.32)
        self.declare_parameter('setpoint',0.0)
        self.Kp = self.get_parameter('kp').value
        self.Ki = self.get_parameter('ki').value
        self.Kd = self.get_parameter('kd').value
        self.setpoint = self.get_parameter('setpoint').value
        self.lastError = 0
        self.IntegralError = 0
        self.DerivativeError = 0
        self.start_time = 0
        self.x_pos = 0.0
        self.y_pos = 0.0

        #Initialize pub
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            'PID_PR', 
            10)
        #Initialize sub
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ball_dist_cm',
            self.sub_callback,
            10)
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        
    def sub_callback(self, msg):
        self.x_pos, self.y_pos = msg.data

        pid_x = self.compute(self.x_pos)
        pid_y = self.compute(self.y_pos)

        pid_output = pid_x, pid_y
        #self.get_logger().info('PID x: %f' % pid_output[0])
        #self.get_logger().info('PID y: %f' % pid_output[1])
        pub_msg = Float32MultiArray(data=pid_output)
        self.publisher_.publish(pub_msg)


    def compute(self, systemValue):
        dt = 0.03
        error = self.setpoint - systemValue
        self.IntegralError += error * dt
        self.IntegralError = np.clip(self.IntegralError, a_min=-5, a_max=5)  # integral windup
        self.DerivativeError = (error - self.lastError) / dt

        output = (-self.Kp * error) + (-self.Ki * self.IntegralError) + (-self.Kd * self.DerivativeError)

        self.lastError = error
        self.start_time = time.time()

        

        return output
    
    def parameter_callback(self, params:list[Parameter]):
        for param in params:
            if param.name == 'kp':
                self.Kp = param.value
            if param.name == 'ki':
                self.Ki = param.value
            if param.name == 'kd':
                self.Kd = param.value
            if param.name == 'setpoint':
                self.setpoint = param.value
    
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=None)
    pidControllerNode_ = pidControllerNode()

    rclpy.spin(pidControllerNode_)
    pidControllerNode_.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
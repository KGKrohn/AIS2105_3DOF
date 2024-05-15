import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import time
import numpy as np
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

"""

class pidControllerNode(Node):
    def __init__(self):
        super().__init__('PID_Node')
        self.load_parameters()
        
        self.state = {
            'x': {'pos': 0.0, 'setpoint': 0, 'lastError': 0, 'integralError': 0, 'derivativeError': 0},
            'y': {'pos': 0.0, 'setpoint': 0, 'lastError': 0, 'integralError': 0, 'derivativeError': 0}
        }

        self.init_publishers()
        self.init_subscribers()
        self.joint_state = JointState(names=['pitch', 'roll', 'ballx', 'bally'], positions=[0.90, 0.3, 0.10, 0.10])
        
    def load_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 0.3),
                ('ki', 0.08),
                ('kd', 0.14),
                ('setpoint_x', 0.0),
                ('setpoint_y', 0.0)
            ]
        )
        self.Kp = self.get_parameter('kp').value
        self.Ki = self.get_parameter('ki').value
        self.Kd = self.get_parameter('kd').value

    def init_publishers(self):
        self.publisher_ = self.create_publisher(Float32MultiArray, 'PID_PR', 10)
        self.publisher_JS = self.create_publisher(JointState, 'joint_states', 5)

    def init_subscribers(self):
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ball_dist_cm',
            self.sub_callback,
            10
        )

    def sub_callback(self, msg):
        self.state['x']['pos'], self.state['y']['pos'] = msg.data
        pid_outputs = {}

        for axis in ['x', 'y']:
            pid_outputs[axis], self.state[axis]['lastError'], self.state[axis]['integralError'], self.state[axis]['derivativeError'] = \
                self.compute_control(self.state[axis]['pos'], self.get_parameter(f'setpoint_{axis}').value, **self.state[axis])

        self.get_logger().info(f'pid_pitch: {pid_outputs["x"]:.2f}, pid_roll: {pid_outputs["y"]:.2f}')
        pub_msg = Float32MultiArray(data=[pid_outputs['x'], pid_outputs['y']])
        self.publisher_.publish(pub_msg)
        self.publisher_JS.publish(self.joint_state)

    def compute_control(self, systemValue, setpoint, lastError, integralError, derivativeError, dt=0.03):
        error = setpoint - systemValue
        integralError += error * dt
        integralError = np.clip(integralError, a_min=-5, a_max=5)
        derivativeError = (error - lastError) / dt
        output = (-self.Kp * error) + (-self.Ki * integralError) + (-self.Kd * derivativeError)
        return output, error, integralError, derivativeError

    def parameter_callback(self, params:list[Parameter]):
        for param in params:
            setattr(self, param.name, param.value)
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = pidControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""

    

class pidControllerNode(Node):
    def __init__(self):
        super().__init__('PID_Node')
        self.load_parameters()
        
        self.state = {
            'x': {'pos': 0.0, 'setpoint': 0, 'lastError': 0, 'integralError': 0, 'derivativeError': 0},
            'y': {'pos': 0.0, 'setpoint': 0, 'lastError': 0, 'integralError': 0, 'derivativeError': 0}
        }

        self.init_publishers()
        self.init_subscribers()
        self.joint_state = JointState(names=['pitch', 'roll', 'ballx', 'bally'], positions=[0.90, 0.3, 0.10, 0.10])
        
    def load_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 0.3),
                ('ki', 0.08),
                ('kd', 0.14),
                ('setpoint_x', 0.0),
                ('setpoint_y', 0.0)
            ]
        )
        self.Kp = self.get_parameter('kp').value
        self.Ki = self.get_parameter('ki').value
        self.Kd = self.get_parameter('kd').value

    def init_publishers(self):
        self.publisher_ = self.create_publisher(Float32MultiArray, 'PID_PR', 10)
        self.publisher_JS = self.create_publisher(JointState, 'joint_states', 5)

    def init_subscribers(self):
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ball_dist_cm',
            self.sub_callback,
            10
        )

    def sub_callback(self, msg):
        self.state['x']['pos'], self.state['y']['pos'] = msg.data
        pid_outputs = {}

        for axis in ['x', 'y']:
            pid_outputs[axis], self.state[axis]['lastError'], self.state[axis]['integralError'], self.state[axis]['derivativeError'] = \
                self.compute_control(self.state[axis]['pos'], self.get_parameter(f'setpoint_{axis}').value, **self.state[axis])

        self.get_logger().info(f'pid_pitch: {pid_outputs["x"]:.2f}, pid_roll: {pid_outputs["y"]:.2f}')
        pub_msg = Float32MultiArray(data=[pid_outputs['x'], pid_outputs['y']])
        self.publisher_.publish(pub_msg)
        self.publisher_JS.publish(self.joint_state)

    def compute_control(self, systemValue, setpoint, lastError, integralError, derivativeError, dt=0.03):
        error = setpoint - systemValue
        integralError += error * dt
        integralError = np.clip(integralError, a_min=-5, a_max=5)
        derivativeError = (error - lastError) / dt
        output = (-self.Kp * error) + (-self.Ki * integralError) + (-self.Kd * derivativeError)
        return output, error, integralError, derivativeError

    def parameter_callback(self, params:list[Parameter]):
        for param in params:
            setattr(self, param.name, param.value)
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = pidControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

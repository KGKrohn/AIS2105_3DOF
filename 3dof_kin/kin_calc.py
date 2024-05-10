import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Generated with the help of ChatGPT
class IKNode(Node):
    def __init__(self):
        super().__init__('IK_node')
        self.declare_parameter('L', 20.0)
        self.declare_parameter('yaw_offset', 20.0)
        self.declare_parameter('servo_arm_length', 5.0)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'PID_PR',
            self.pitch_roll_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'servo_angles', 10)

    def pitch_roll_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn("Received invalid message. Expected 2 values, received %d", len(msg.data))
            return

        pitch, roll = msg.data

        transformed_values = self.perform_transformation(pitch, roll, 0)

        servo_angles = self.calculate_servo_angles(transformed_values)
        
        servo_msg = Float32MultiArray(data=servo_angles)
        self.publisher_.publish(servo_msg)

    def rotation_matrix(self, roll, pitch, yaw):

        phi = np.deg2rad(roll)
        theta = np.deg2rad(pitch)
        psi = np.deg2rad(yaw)

        x_rot = np.array([
        [1, 0, 0],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]
        ])

        y_rot = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
        ])

        z_rot = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
        ])

        return z_rot @ y_rot @ x_rot


    def perform_transformation(self, pitch, roll, height):

        L = self.get_parameter('L').value
        yaw_offset = self.get_parameter('yaw_offset').value
        z = height

        P1 = np.array([[L/2], [L/(2*np.sqrt(3))], [0], [1]])
        P2 = np.array([[-L/2], [L/(2*np.sqrt(3))], [0], [1]])
        P3 = np.array([[0], [-L/np.sqrt(3)], [0], [1]])

        rotation_matrix = self.rotation_matrix(roll, pitch, yaw_offset)
        translation_vector = np.array([0, 0, z])

        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix 
        transformation_matrix[:3, 3] = translation_vector

        P1_new = transformation_matrix @ P1
        P2_new = transformation_matrix @ P2
        P3_new = transformation_matrix @ P3

        transformed_values = P1_new[2,0], P2_new[2,0], P3_new[2,0]

        return transformed_values

    def calculate_servo_angles(self, transformed_values):

        r = self.get_parameter('servo_arm_length').value

        servo1 = np.rad2deg(np.arcsin(np.clip(transformed_values[0],-5,5)/r))
        servo2 = np.rad2deg(np.arcsin(np.clip(transformed_values[1],-5,5)/r))
        servo3 = np.rad2deg(np.arcsin(np.clip(transformed_values[2],-5,5)/r))

        # Clip all servo angles between -45 and 45 since this is the working area of our servos
        servo_angles = np.clip(servo2,-45, 45), np.clip(servo3,-45, 45), np.clip(servo1,-45, 45)

        return servo_angles

def main(args=None):
    rclpy.init(args=args)
    IK_node = IKNode()
    rclpy.spin(IK_node)
    IK_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

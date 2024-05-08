import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class BallToCenti(Node):
    def __init__(self):
        super().__init__('ballToCentiConversion')
        self.subscription_ball_pos = self.create_subscription(
            Float32MultiArray,
            'raw_ball_pos',
            self.ball_position_callback,
            10)
        self.subscription_plate_center_pos = self.create_subscription(
            Float32MultiArray,
            'plate_center_pos',
            self.plate_center_callback,
            10)
        self.publisher_ball_dist_cm = self.create_publisher(
            Float32MultiArray,
            'ball_dist_cm',
            10)

        # Initialize variables
        self.ball_x = None
        self.ball_y = None
        self.plate_center_x = 0
        self.plate_center_y = 0
        self.camera_to_plate_distance = 60
        self.camera_matrix = np.array([[572.587635, 0.0, 301.381129],
                                        [0.0, 576.260137, 236.044851],
                                        [0.0, 0.0, 1.0]])
        self.camera_to_plate_matrix = np.array([[1, 0, 0, -self.plate_center_x],
                                                [0, 1, 0, -self.plate_center_y],
                                                [0, 0, 1, self.camera_to_plate_distance],
                                                [0, 0, 0, 1]])
        self.big_pi_matrix = np.array([[self.camera_to_plate_distance, 0, 0],
                                      [0, self.camera_to_plate_distance, 0],
                                      [0, 0, self.camera_to_plate_distance],
                                      [0, 0, 1]])
        
    def plate_center_callback(self, msg):
        plate_center_x, plate_center_y = msg.data
        self.plate_center_x = plate_center_x
        self.plate_center_y = plate_center_y

    def ball_position_callback(self, msg):
        # Extract pixel coordinates from message
        ball_x, ball_y = msg.data

        # Convert pixel coordinates to camera coordinates
        x_cord, y_cord = self.pixel_to_camera_coordinates(ball_x, ball_y)
        plate_x, plate_y = self.pixel_to_camera_coordinates(self.plate_center_x, self.plate_center_y)

        # Convert camera coordinates to real-world coordinates (optional)

        # Publish converted coordinates
        ball_dist_cm_msg = Float32MultiArray()
        ball_dist_cm_msg.data = x_cord - plate_x, y_cord - plate_y
        self.publisher_ball_dist_cm.publish(ball_dist_cm_msg)

    def pixel_to_camera_coordinates(self, u, v):
        r = np.array([u, v, 1])
        camera_coords = self.camera_to_plate_matrix @ self.big_pi_matrix @ np.linalg.inv(self.camera_matrix) @ r

        return float(camera_coords[0]), float(camera_coords[1])

def main(args=None):
    rclpy.init(args=args)
    visualizer_node = BallToCenti()
    rclpy.spin(visualizer_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
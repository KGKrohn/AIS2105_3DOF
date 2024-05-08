import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

# Draw functions made with the help of ChatGPT
class BallVisualizerNode(Node):
    def __init__(self):
        super().__init__('ball_visualizer_node')
        self.subscription_ball_pos = self.create_subscription(
            Float32MultiArray,
            'raw_ball_pos',
            self.ball_position_callback,
            10)
        self.subscription_image = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        self.publisher_image = self.create_publisher(
            Image,
            'drawed_image',
            10)
        self.bridge = CvBridge()

        # Initialize variables
        self.prev_positions = []
        self.prev_positions_max_len = 15 

    def ball_position_callback(self, msg):
        ball_x, ball_y = msg.data[0], msg.data[1]
        self.prev_positions.append((ball_x, ball_y))
        self.last_ball_time = self.get_clock().now()

        # Check if the number of previous positions exceeds the maximum length
        if len(self.prev_positions) > self.prev_positions_max_len:
            self.prev_positions = self.prev_positions[-self.prev_positions_max_len:]

    def draw_trail_and_velocity_vector(self, image):
        # Draw trail
        for i in range(1, len(self.prev_positions)):
            cv2.line(image, (int(self.prev_positions[i - 1][0]), int(self.prev_positions[i - 1][1])),
                     (int(self.prev_positions[i][0]), int(self.prev_positions[i][1])), (255, 0, 0), 2)

        # Draw velocity vector
        if len(self.prev_positions) >= 2:
            vector = [self.prev_positions[-1][0] - self.prev_positions[-2][0], 
                      self.prev_positions[-1][1] - self.prev_positions[-2][1]]
            cv2.arrowedLine(image, (int(self.prev_positions[-2][0]), int(self.prev_positions[-2][1])),
                            (int(self.prev_positions[-2][0] + vector[0] * 8), int(self.prev_positions[-2][1] + vector[1] * 8)),
                            (128, 0, 128), 3)

    def image_callback(self, msg):
        if self.prev_positions:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.draw_trail_and_velocity_vector(image)

            # Publish image with vector and trail
            image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.publisher_image.publish(image_msg)

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    visualizer_node = BallVisualizerNode()
    visualizer_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

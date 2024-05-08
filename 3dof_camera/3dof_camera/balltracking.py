import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2

class ballTrackingNode(Node):

    def __init__(self):
        super().__init__('ballTrackingNode')

        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10)
        self.subscription

        self.publisher_blur = self.create_publisher(
            Image,
            'gaus_blur_image',
            10)
        
        self.publisher_edge  = self.create_publisher(
            Image,
            'edged_image',
            10)
        
        self.publisher_hough = self.create_publisher(
            Image,
            'hough_image',
            10)
        
        self.publisher_raw_ball_pos = self.create_publisher(
            Float32MultiArray,
            'raw_ball_pos',
            10)
        
        self.publisher_plate_center_pos = self.create_publisher(
            Float32MultiArray,
            'plate_center_pos',
            10)
  
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('debug_mode', False)

        self.declare_parameter('gaussian_blur_radius', 5)

        self.declare_parameter('canny_lower_threshold', 190)
        self.declare_parameter('canny_upper_threshold', 260)

        self.declare_parameter('hough_dp', 1.2)
        self.declare_parameter('hough_min_dist', 100)
        self.declare_parameter('hough_param1', 100)
        self.declare_parameter('hough_param2', 30)
        self.declare_parameter('hough_min_radius', 5)
        self.declare_parameter('hough_max_radius', 40)
        self.declare_parameter('plate_hough_min_radius', 190)
        self.declare_parameter('plate_hough_max_radius', 220)

    def image_callback(self, msg):
        debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value

        raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        blur_radius = self.get_parameter('gaussian_blur_radius').value

        # Ensure blur radius is odd
        if blur_radius % 2 == 0:
            blur_radius += 1
            self.set_parameters([rclpy.parameter.Parameter('gaussian_blur_radius', rclpy.Parameter.Type.INTEGER, blur_radius)])
            print("Please pass only odd values")

        blurred_image = cv2.GaussianBlur(raw_image, (int(blur_radius), int(blur_radius)), 0)

        canny_lower_threshold = self.get_parameter('canny_lower_threshold').value
        canny_upper_threshold = self.get_parameter('canny_upper_threshold').value

        edged_image = cv2.Canny(blurred_image, canny_lower_threshold, canny_upper_threshold)

        # Apply Hough circles detection
        dp = self.get_parameter('hough_dp').value
        min_dist = self.get_parameter('hough_min_dist').value
        param1 = self.get_parameter('hough_param1').value
        param2 = self.get_parameter('hough_param2').value
        min_radius = self.get_parameter('hough_min_radius').value
        max_radius = self.get_parameter('hough_max_radius').value
        plate_min_radius = self.get_parameter('plate_hough_min_radius').value
        plate_max_radius = self.get_parameter('plate_hough_max_radius').value

        hough_image = np.copy(raw_image)

        plate = cv2.HoughCircles(edged_image, cv2.HOUGH_GRADIENT, dp, min_dist, param1=param1, param2=param2, minRadius=plate_min_radius, maxRadius=plate_max_radius)
        
        if plate is not None:
            i = plate[0, 0]
            if debug_mode:
                # Draw the outer circle
                cv2.circle(hough_image, (int(i[0]), int(i[1])), int(i[2]), (0, 255, 0), 2)
                # Draw the center of the circle
                cv2.circle(hough_image, (int(i[0]), int(i[1])), 2, (0, 0, 255), 3)

            # Radius of the circular mask
            radius = int(plate[0, 0, 2] + 20)

            # Circular mask
            mask_plat = np.zeros(edged_image.shape, dtype='uint8')
            cv2.circle(mask_plat, (int(plate[0, 0, 0]), int(plate[0, 0, 1])), radius, (255, 255, 255), -1)

            plate_center_msg = Float32MultiArray()
            plate_center_msg.data = [float(i[0]), float(i[1])]
            self.publisher_plate_center_pos.publish(plate_center_msg)

        else:
            # Create an empty mask
            mask_plat = np.zeros(edged_image.shape, dtype='uint8')

        # Apply mask
        masked_edged_image = cv2.bitwise_and(edged_image, edged_image, mask=mask_plat)

        circles = cv2.HoughCircles(masked_edged_image, cv2.HOUGH_GRADIENT, dp, min_dist, param1=param1, param2=param2, minRadius=min_radius, maxRadius=max_radius)

        if circles is not None:
            i = circles[0, 0]
            # Draw the outer circle
            cv2.circle(hough_image, (int(i[0]), int(i[1])), int(i[2]), (0, 255, 0), 2)
            # Draw the center of the circle
            cv2.circle(hough_image, (int(i[0]), int(i[1])), 2, (0, 0, 255), 3)

            ball_pos_msg = Float32MultiArray()
            ball_pos_msg.data = [float(i[0]), float(i[1])]
            self.publisher_raw_ball_pos.publish(ball_pos_msg)

        # Publish Hough circles image
        ros2_hough_image_msg = self.bridge.cv2_to_imgmsg(hough_image, encoding='bgr8')
        self.publisher_hough.publish(ros2_hough_image_msg)

        if debug_mode:
            ros2_blurred_image_msg = self.bridge.cv2_to_imgmsg(blurred_image, encoding='bgr8')
            self.publisher_blur.publish(ros2_blurred_image_msg)

            ros2_edged_image_msg = self.bridge.cv2_to_imgmsg(edged_image, encoding='mono8')
            self.publisher_edge.publish(ros2_edged_image_msg)


def main(args=None):
    rclpy.init(args=args)
    ball_tracking_node = ballTrackingNode()
    rclpy.spin(ball_tracking_node)
    ball_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
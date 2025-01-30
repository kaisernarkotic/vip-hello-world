import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    """ROS2 Node that detects whether an object is red or blue."""

    def __init__(self):
        super().__init__('camera_node')

        # Logger
        self.logger = self.get_logger()

        # Publisher (publishes detected color)
        self.color_pub = self.create_publisher(String, '/detected_color', 10)

        # Subscriber (receives images from ZED camera)
        self.image_subscriber = self.create_subscription(
            Image,
            '/zed/zed_node/stereo_raw/image_raw_color',  # Update topic as needed
            self.image_callback,
            10
        )

        # OpenCV Bridge
        self.bridge = CvBridge()

        # Define color thresholds in HSV
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 120, 70])
        self.upper_red2 = np.array([179, 255, 255])

        self.lower_blue = np.array([100, 150, 50])
        self.upper_blue = np.array([140, 255, 255])

        self.logger.info('CameraNode has started.')

    def image_callback(self, msg):
        """Processes an image and determines if an object is red or blue."""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Create masks for red and blue
            red_mask = cv2.inRange(hsv_image, self.lower_red1, self.upper_red1) | \
                       cv2.inRange(hsv_image, self.lower_red2, self.upper_red2)
            blue_mask = cv2.inRange(hsv_image, self.lower_blue, self.upper_blue)

            # Count the number of red and blue pixels
            red_pixels = np.sum(red_mask > 0)
            blue_pixels = np.sum(blue_mask > 0)

            # Determine the dominant color
            detected_color = "none"
            if red_pixels > 500:
                detected_color = "red"
            elif blue_pixels > 500:
                detected_color = "blue"
            else:
                detected_color = "none"

            # Publish detected color
            self.color_pub.publish(String(data=detected_color))
            self.logger.info(f"Detected color: {detected_color}")

        except Exception as e:
            self.logger.error(f"Error processing image: {e}")

def main(args=None):
    """Main function to run the CameraNode."""
    rclpy.init(args=args)
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.info("CameraNode interrupted by user.")
    finally:
        self.zed.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

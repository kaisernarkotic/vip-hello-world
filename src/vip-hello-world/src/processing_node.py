import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ProcessingNode(Node):
    """Node that processes the detected color form the camera."""
    def __init__(self):
        super().__init__('processing_node')

        #Logger
        self.logger = self.get_logger()

        self.color_subscriber = self.create_subscription(String, '/detected_color', self.color_callback, 10)

        self.drive_publisher = self.create_publisher(AckermannDrive, '/turn_direction', 10)

        self.logger.info("Processing Node is running and subscribed to /detected_color.")

    def color_callback(self, msg):
        """Processes the received color data."""
        detected_color = msg.data
        self.logger.info(f"Received color: {detected_color}")

        drive_msg = AckeramannDrive()
        drive_msg.speed = 0

        # Process the color and decide next action
        if detected_color == "red":
            drive_msg.steering_angle = 0.5
            self.logger.info("Red detected! Preparing to turn left.")
        elif detected_color == "blue":
            drive_msg.steering_angle = -0.5
            self.logger.info("Blue detected! Preparing to turn right.")
        else: # color is none
            drive_msg.steering_angle = 0
            self.logger.info("No significant color detected. Continuing straight.")
        
        self.drive_published.publish(drive_msg)

def main(args=None):
    """Main function to run the Processing Node."""
    rclpy.init(args=args)
    node = ProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.info("Processing Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

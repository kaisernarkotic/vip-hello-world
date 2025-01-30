import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(AckermannDrive, '/turn_direction', self.control_car, 10)
        self.publisher = self.create_publisher(Twist, '/car/steering_cmd', 10)

    def control_car(self, msg):
        twist = Twist()
        twist.angular.z = msg.steering_angle

        self.publisher.publish(twist)
        self.get_logger().info(f"Sent steering command: {msg.data}")

def main(args=None):
    """Main function to run the Control Node."""
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.info("Control Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

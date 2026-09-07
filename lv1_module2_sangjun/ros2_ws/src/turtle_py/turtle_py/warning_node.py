import rclpy
import math


from rclpy.node import Node

from std_msgs.msg import Float32


class WarningNode(Node):

    def __init__(self):
        super().__init__('warning_node')

        self.distance_subscriber = self.create_subscription(
            Float32,
            '/turtle_distance',
            self.distance_callback,
            10
        )

        self.get_logger().info('Warning node started')

    def distance_callback(self, msg):
        distance = msg.data

        if distance > 2.5:
            self.get_logger().warn(
                f'Turtle is far from origin: {distance:.2f} m'
            )


def main(args=None):
    rclpy.init(args=args)

    node = WarningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
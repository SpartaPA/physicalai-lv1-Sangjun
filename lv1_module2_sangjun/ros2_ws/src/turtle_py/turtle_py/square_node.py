import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class SquareNode(Node):

    def __init__(self):
        super().__init__('square_node')

        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # 주행 상태
        self.state = 'forward'

        # 각 상태가 시작된 시간
        self.state_start_time = self.get_clock().now()

        # 0.1초마다 상태 확인
        self.timer = self.create_timer(
            0.1,
            self.move_turtle
        )

        self.get_logger().info('Square node started')

    def move_turtle(self):
        cmd = Twist()

        elapsed_time = (
            self.get_clock().now() - self.state_start_time
        ).nanoseconds / 1e9

        if self.state == 'forward':
            # 직진
            cmd.linear.x = 1.0
            cmd.angular.z = 0.0

            # 2초 직진 후 회전
            if elapsed_time >= 2.0:
                self.state = 'turn'
                self.state_start_time = self.get_clock().now()

        elif self.state == 'turn':
            # 제자리에서 회전
            cmd.linear.x = 0.0
            cmd.angular.z = 1.0

            # 약 90도 회전 후 직진
            if elapsed_time >= math.pi / 2:
                self.state = 'forward'
                self.state_start_time = self.get_clock().now()

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = SquareNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            stop_cmd = Twist()
            node.publisher.publish(stop_cmd)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
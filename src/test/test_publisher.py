import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class JointTrajectoryTestPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_test_publisher')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/test/joint_trajectory',
            10
        )

        self.timer_period = 1.0  # 1 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_msg)

        self.get_logger().info('Publishing JointTrajectory to /test/joint_trajectory')

    def publish_msg(self):
        msg = JointTrajectory()

        # joint 이름 (순서 중요하지만 테스트라 의미 없음)
        msg.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        point = JointTrajectoryPoint()

        # 임의의 값 (rad 기준이지만 테스트용)
        point.positions = [0.0, 0.1, 0.2, 0.0, 0.0, 0.0]
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        point.effort = []

        # 2초 후 도달한다는 의미
        point.time_from_start = Duration(sec=2, nanosec=0)

        msg.points.append(point)

        self.pub.publish(msg)
        self.get_logger().info('JointTrajectory message published')


def main():
    rclpy.init()
    node = JointTrajectoryTestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

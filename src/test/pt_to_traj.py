
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

def deg2rad(d): return d * math.pi / 180.0

class PointStreamToTrajectory(Node):
    def __init__(self):
        super().__init__('pointstream_to_trajectory')

        self.sub = self.create_subscription(
            JointTrajectoryPoint,
            '/test/traj_point',
            self.on_point,
            10
        )

        self.pub = self.create_publisher(
            JointTrajectory,
            '/test/joint_trajectory',
            10
        )

        self.points = []
        self.timer = self.create_timer(40.0, self.publish_trajectory)

        self.joint_names = [
            'joint_1','joint_2','joint_3',
            'joint_4','joint_5','joint_6'
        ]

    def on_point(self, msg: JointTrajectoryPoint):
        q = list(msg.positions)
        q[1] += deg2rad(14.0)
        msg.positions = q
        
        self.points.append(msg)

    def publish_trajectory(self):
        if not self.points:
            return

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names
        traj.points = self.points

        self.pub.publish(traj)

        self.points = []  # reset


def main():
    rclpy.init()
    node = PointStreamToTrajectory()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

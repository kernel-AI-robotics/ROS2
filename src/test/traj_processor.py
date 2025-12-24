import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

def deg2rad(d): 
    return d * math.pi / 180.0

class TrajProcessor(Node):
    def __init__(self):
        super().__init__('joint_trajectory_processor')

        self.sub_topic = '/test/joint_trajectory'
        self.pub_topic = '/dsr01/dsr_joint_trajectory/joint_trajectory'

        self.offset = deg2rad(14.0)  # joint_2에 더함

        self.pub = self.create_publisher(JointTrajectory, self.pub_topic, 10)
        self.sub = self.create_subscription(JointTrajectory, self.sub_topic, self.on_traj, 10)

        self.get_logger().info(f"Sub: {self.sub_topic}")
        self.get_logger().info(f"Pub: {self.pub_topic}")

    def on_traj(self, msg: JointTrajectory):
        print("hi")
        # joint_2가 없으면 패스
        if 'joint_2' not in msg.joint_names:
            self.get_logger().warn("joint_2 not found in incoming JointTrajectory.joint_names")
            return

        j2_idx = msg.joint_names.index('joint_2')

        out = JointTrajectory()
        out.header = msg.header            # stamp도 그대로 전달 (원하면 now로 바꿔도 됨)
        out.joint_names = list(msg.joint_names)

        out_points = []
        for p in msg.points:
            q = list(p.positions) if p.positions else []
            if len(q) <= j2_idx:
                self.get_logger().warn("positions length shorter than joint_names; skipping point")
                continue

            #q[j2_idx] += self.offset

            np = JointTrajectoryPoint()
            np.positions = q
            np.velocities = list(p.velocities) if p.velocities else []
            np.accelerations = list(p.accelerations) if p.accelerations else []
            np.effort = list(p.effort) if p.effort else []
            np.time_from_start = p.time_from_start  # 그대로 유지

            out_points.append(np)

        if not out_points:
            self.get_logger().warn("No valid points to publish.")
            return

        out.points = out_points
        self.pub.publish(out)

def main():
    rclpy.init()
    node = TrajProcessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

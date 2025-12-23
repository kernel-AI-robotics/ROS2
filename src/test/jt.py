import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import math

def deg2rad(d): return d * math.pi / 180.0

class SlowJTCTester(Node):
    def __init__(self):
        super().__init__('slow_jtc_tester')

        # ✅ 네 컨트롤러 토픽으로 바꿔서 쓰면 됨:
        # 예) '/dsr01/dsr_joint_trajectory/joint_trajectory'
        self.cmd_topic = '/dsr01/dsr_joint_trajectory/joint_trajectory'

        self.pub = self.create_publisher(JointTrajectory, self.cmd_topic, 10)
        self.sub = self.create_subscription(JointState, '/dsr01/joint_states', self.on_js, 10)

        self.sent = False
        self.q0 = None  # current joints (rad), ordered joint_1..6

        self.get_logger().info(f'Publishing to: {self.cmd_topic}')
        self.get_logger().info('Waiting joint_states...')

    def on_js(self, msg: JointState):
        if self.sent:
            return

        # joint_states name 순서가 섞여서 오므로, joint_1..6으로 재정렬
        mp = {n: p for n, p in zip(msg.name, msg.position)}  # p: rad
        order = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
        if not all(j in mp for j in order):
            return

        self.q0 = [mp[j] for j in order]
        self.send_trajectory(order, self.q0)
        self.sent = True

        self.get_logger().info('Trajectory published once. (Node will keep running)')

    def send_trajectory(self, joint_names, q_start):
        traj = JointTrajectory()
        traj.joint_names = joint_names

        # 목표: joint_2를 +5deg(=0.087rad) 천천히
        q_goal = list(q_start)
        q_goal[1] -= deg2rad(5.0)

        # 1) 시작점 (t=0)
        p0 = JointTrajectoryPoint()
        p0.positions = q_start
        p0.velocities = [0.0]*6
        p0.accelerations = [0.0]*6
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0

        # 2) 목표점 (t=6s)  ← 더 느리게 하고 싶으면 10~15s로 늘려
        p1 = JointTrajectoryPoint()
        p1.positions = q_goal
        p1.velocities = [0.0]*6
        p1.accelerations = [0.0]*6
        p1.time_from_start.sec = 6
        p1.time_from_start.nanosec = 0

        traj.points = [p0, p1]

        self.pub.publish(traj)

def main():
    rclpy.init()
    node = SlowJTCTester()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

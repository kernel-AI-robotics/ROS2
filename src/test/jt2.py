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

        # :흰색_확인_표시: (중요) 기준 시간 명시 (없으면 일부 구현에서 "뚝뚝" 나올 수 있음)
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = ''  # 보통 비움

        # 목표: joint_2를 -5deg 천천히
        q_goal = list(q_start)
        q_goal[1] -= deg2rad(20.0)

        # --- 부드러운 궤적 생성 파라미터 ---
        T = 5.0        # 총 이동시간(s) : 더 느리게 = 12~20으로
        dt = 0.05       # 포인트 간격(s) : 0.02~0.05 추천
        N = int(T / dt) + 1
        dq = [q_goal[i] - q_start[i] for i in range(6)]

        # quintic time-scaling: s(u)=10u^3-15u^4+6u^5 (u=t/T)
        # ds/du=30u^2-60u^3+30u^4
        # d2s/du2=60u-180u^2+120u^3
        points = []
        for k in range(N):
            t = k * dt
            u = min(max(t / T, 0.0), 1.0)

            s  = 10*u**3 - 15*u**4 + 6*u**5
            ds = (30*u**2 - 60*u**3 + 30*u**4) / T
            d2s = (60*u - 180*u**2 + 120*u**3) / (T*T)

            p = JointTrajectoryPoint()
            p.positions = [q_start[i] + s * dq[i] for i in range(6)]
            p.velocities = [ds * dq[i] for i in range(6)]
            p.accelerations = [d2s * dq[i] for i in range(6)]
            # effort는 보통 비워도 됨
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)

            points.append(p)

        traj.points = points
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

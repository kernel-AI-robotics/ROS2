import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

import math

def deg2rad(d): return d * math.pi / 180.0

class TrajPointPublisher(Node):
    def __init__(self):
        super().__init__('pt_publisher')

        # 통신을 위한 토픽명
        self.pub_topic = '/test/traj_point'

        self.pub = self.create_publisher(JointTrajectoryPoint, self.pub_topic, 10)
        self.sub = self.create_subscription(JointState, '/dsr01/joint_states', self.on_js, 10)

        self.sent = False
        self.q0 = None  # current joints (rad), ordered joint_1..6

        self.get_logger().info(f'Publishing to: {self.pub_topic}')
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
        self.q0[1] -= deg2rad(14.0)
        self.send_points(order, self.q0)
        self.sent = True

        self.get_logger().info('Trajectory published once. (Node will keep running)')

    def send_points(self, joint_names, q_start):

        # --- 원형(조인트 공간) 파라미터 ---
        T = 6.0          # 한 바퀴 도는 시간(s)  -> 더 천천히: 20~30
        dt = 0.02         # 포인트 간격(s)       -> 0.01~0.05
        N = int(T / dt) + 1

        A = deg2rad(20.0)   # joint2/3 진폭 (deg)
        w  = 2.0 * math.pi / T

        # q_start가 원 궤도의 중심이 되도록
        q_start[2] -= A

        for k in range(N):
            t = k * dt

            # joint_2 = sin, joint_3 = cos  => (joint_2, joint_3) 평면에서 원
            s = math.sin(w * t)
            c = math.cos(w * t)

            p = JointTrajectoryPoint()

            q = list(q_start)
            q[1] = q_start[1] + A * s   # joint_2
            q[2] = q_start[2] + A * c   # joint_3
            p.positions = q

            # 속도/가속도 (부드럽게)
            dq = [0.0]*6
            ddq = [0.0]*6
            dq[1]  = A * w * c
            dq[2]  = -A * w * s
            ddq[1] = -A * w*w * s
            ddq[2] = -A * w*w * c
            p.velocities = dq
            p.accelerations = ddq

            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)

            self.pub.publish(p)
            time.sleep(0.1)

def main():
    rclpy.init()
    node = TrajPointPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

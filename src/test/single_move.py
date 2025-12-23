import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint

def main():
    # ROS2 초기화
    rclpy.init()

    # 노드 생성
    node = Node('single_move')

    # 서비스 클라이언트 생성
    client = node.create_client(MoveJoint, '/dsr01/motion/move_joint')

    # 서비스 대기
    print("서비스 연결 중...")
    if not client.wait_for_service(timeout_sec=10.0):
        print("✗ 서비스를 찾을 수 없습니다!")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("✓ 서비스 연결 완료")

    # 요청 생성
    request = MoveJoint.Request()
    request.pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
    request.vel = 60.0
    request.acc = 60.0

    # 서비스 호출
    print("로봇 이동 시작...")
    print(f"목표 위치: {request.pos}")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        print("✓ 이동 완료!")
    else:
        print("✗ 이동 실패!")

    # 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

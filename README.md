# 3조 ROS2 repo

## 설치
```bash
git clone --recurse-submodules https://github.com/kernel-AI-robotics/ROS2.git
cd ROS2
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# --recurse-submodules 옵션 안 했을 경우
git submodule update --init --recursive
```
## launch 하기
1. 도커 실행(터미널 1)
```bash
# 기존 컨테이너 모두 종료
sudo docker stop $(sudo docker ps -q) 2>/dev/null
# Docker 정리
sudo docker system prune -f
# 다시 실행
sudo docker run -it --rm -p 12345:12345 doosanrobot/dsr_emulator:3.0.1
```
2. launch파일 실행
```bash
# 실제 로봇 연결
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=110.120.1.63

# 가상 환경 실행(rviz)
ros2 launch e0509_gripper_description bringup.launch.py mode:=virtual
```

## joint_trajectory 컨트롤러 실행시키기
```bash

ros2 run controller_manager spawner dsr_joint_trajectory -c /dsr01/controller_manager

# 컨트롤러 확인
ros2 control list_controllers -c /dsr01/controller_manager 

# 토픽 확인(선택)
ros2 topic echo /dsr01/dsr_joint_trajectory/joint_trajectory 
```

---

## 그리퍼 움직이기
```bash
# 그리퍼 열기
ros2 service call /dsr01/gripper/open std_srvs/srv/Trigger

# 그리퍼 닫기
ros2 service call /dsr01/gripper/close std_srvs/srv/Trigger

# Stroke 값으로 제어 (0=열림, 700=완전히 닫힘)
ros2 topic pub /dsr01/gripper/position_cmd std_msgs/msg/Int32 "{data: 350}" --once
```

## test파일 실행하기 (movej)
```bash
# 한 번 움직이기([0, 14, 90, 0, 90, 0])
python3 <ROS2>/src/test/single_move.py
# 왕복 5번 움직이기([0, 0, 90, 0, 90, 0] ~ [0, 14, 90, 0, 90, 0])
python3 <ROS2>/src/test/multiple_move.py
```

## test파일 실행하기 (joint_trajectory)
```bash
# python 코드 실행
python3 <ROS2>/src/test/traj_processor.py
python3 <ROS2>/python3 src/test/jt_publisher.py

# 토픽 확인하기
ros2 topic echo /test/joint_trajectory
ros2 topic echo /dsr01/dsr_joint_trajectory/joint_trajectory 
```

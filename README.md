git clone --recurse-submodules https://github.com/kernel-AI-robotics/ROS2.git

cd ROS2

colcon build --symlink-install

source install/setup.bash

-----------------

ros2 run controller_manager spawner dsr_joint_trajectory -c /dsr01/controller_manager

ros2 control switch_controllers -c /dsr01/controller_manager --deactivate dsr_controller2

ros2 control list_controllers -c /dsr01/controller_manager 

ros2 topic echo /dsr01/dsr_joint_trajectory/joint_trajectory 

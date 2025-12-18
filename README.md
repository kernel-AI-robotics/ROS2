git clone --recurse-submodules https://github.com/kernel-AI-robotics/ROS2.git

cd ROS2

colcon build --symlink-install

source install/setup.bash

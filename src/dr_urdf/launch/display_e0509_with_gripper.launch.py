import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('dr_urdf')
    xacro_file = os.path.join(pkg_path, 'urdf', 'e0509_with_gripper.xacro')
    rviz_config = os.path.join(pkg_path, 'rviz', 'e0509_with_gripper.rviz')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    params = {'robot_description': robot_description}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[params],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])

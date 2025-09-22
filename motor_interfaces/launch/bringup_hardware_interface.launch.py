from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('motor_interfaces')

    urdf_file = os.path.join(pkg_share, 'urdf', 'hardware_interface.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'diff_drive_controller.yaml')

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {"robot_description": open(urdf_file).read()},
                controllers_file,   # <- rất quan trọng
            ],
            output='screen'
        ),

        # Spawner cho broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        ),

        # Spawner cho diff drive
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
        ),
    ])

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue   # <-- thêm dòng này
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('motor_interfaces')

    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'diff_drive_controller.yaml')
    # robot_description = ParameterValue(Command(['xacro', urdf_path]),value_type=str)

    robot_description = ParameterValue(
    Command(['xacro ' + urdf_path]),
    value_type=str
)



    # Node publish TF từ URDF
    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{'robot_description': robot_description}],
    # )

    # # Node control chính
    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         {'robot_description': robot_description},   # nạp robot_description
    #         controllers_file                           # nạp config controller yaml
    #     ],
    #     output='screen'
    # )

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[controllers_file],
    # )  
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_file
        ],
        output="screen"
    )

    # Spawner cho broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawner cho diff drive
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        # robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])

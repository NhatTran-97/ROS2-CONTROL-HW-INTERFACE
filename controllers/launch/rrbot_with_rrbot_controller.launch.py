import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():

    rrbot_description_path = os.path.join(
        get_package_share_directory('rrbot_unit2'))

    xacro_file = os.path.join(rrbot_description_path,
                              'urdf',
                              'rrbot.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot'],
                        output='screen')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rrbot_controller", "-c", "/controller_manager"],
    )


    return LaunchDescription([
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_entity,
                on_completion=[
                    LogInfo(
                        msg='Spawn finished, waiting 10 seconds to start controllers.'),
                    TimerAction(
                        period=10.0,
                        actions=[joint_state_broadcaster_spawner],
                    )
                ]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        ),
        spawn_entity,
        node_robot_state_publisher,
    ])
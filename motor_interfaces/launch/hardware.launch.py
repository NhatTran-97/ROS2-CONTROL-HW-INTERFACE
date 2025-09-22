from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim = LaunchConfiguration("is_sim")
    
    return LaunchDescription([
        # Declare sim mode argument
        DeclareLaunchArgument("is_sim", default_value="false"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": False
            }],
            arguments=[
                PathJoinSubstitution([
                    FindPackageShare("motor_interfaces"), 
                    "urdf",
                    "hardware_interface.xacro" 
                ]),
                "is_sim:=false"
            ],
        ),

        # Controller Manager (cần có robot_description)
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("your_package_name"),
                    "config",
                    "ros2_controllers.yaml"  # 🔁 File config controller
                ]),
                {
                    "robot_description": Command([
                        "xacro ",
                        PathJoinSubstitution([
                            FindPackageShare("your_package_name"),
                            "urdf",
                            "your_robot.urdf.xacro"
                        ]),
                        " is_sim:=false"
                    ])
                },
            ],
            output="screen",
        ),

        # Spawn diff_drive_controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        # (Optional) Spawn joint_state_broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ])

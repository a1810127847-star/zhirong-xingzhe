from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vision_params_file = LaunchConfiguration("vision_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vision_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("zhirong_vision"),
                        "config",
                        "vision.yaml",
                    ]
                ),
                description="Vision detector parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time.",
            ),
            Node(
                package="zhirong_vision",
                executable="color_qr_detector",
                name="color_qr_detector",
                output="screen",
                parameters=[
                    vision_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )

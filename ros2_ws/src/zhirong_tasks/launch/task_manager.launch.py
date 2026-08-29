from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    task_config = LaunchConfiguration("task_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    vision_armed = LaunchConfiguration("vision_armed")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "task_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("zhirong_tasks"),
                        "config",
                        "tasks.yaml",
                    ]
                ),
                description="Task catalog and vision mapping configuration.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time.",
            ),
            DeclareLaunchArgument(
                "vision_armed",
                default_value="false",
                description="Allow stable vision events to enqueue navigation tasks.",
            ),
            Node(
                package="zhirong_tasks",
                executable="task_manager",
                name="task_manager",
                output="screen",
                parameters=[
                    {
                        "task_config": task_config,
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                        "vision_armed": ParameterValue(
                            vision_armed,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )

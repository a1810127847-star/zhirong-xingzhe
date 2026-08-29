from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("zhirong_bringup"),
                    "launch",
                    "simulation.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gui": gui,
            "rviz": "true",
            "rviz_config": PathJoinSubstitution(
                [
                    FindPackageShare("zhirong_bringup"),
                    "rviz",
                    "zhirong_mapping.rviz",
                ]
            ),
            "slam": "true",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Launch the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time.",
            ),
            simulation_launch,
        ]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    navigation_rviz = LaunchConfiguration("navigation_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    vision_armed = LaunchConfiguration("vision_armed")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("zhirong_bringup"),
                    "launch",
                    "navigation.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gui": gui,
            "navigation_rviz": navigation_rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("zhirong_vision"),
                    "launch",
                    "vision.launch.py",
                ]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    tasks = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("zhirong_tasks"),
                    "launch",
                    "task_manager.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "vision_armed": vision_armed,
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
                "navigation_rviz",
                default_value="true",
                description="Launch the navigation RViz2 interface.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time.",
            ),
            DeclareLaunchArgument(
                "vision_armed",
                default_value="false",
                description=(
                    "Allow stable color/QR events to enqueue tasks. "
                    "Defaults off to prevent unexpected motion."
                ),
            ),
            navigation,
            TimerAction(period=2.0, actions=[vision]),
            TimerAction(period=5.0, actions=[tasks]),
        ]
    )

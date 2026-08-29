from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    slam = LaunchConfiguration("slam")
    slam_params_file = LaunchConfiguration("slam_params_file")
    safety_monitor = LaunchConfiguration("safety_monitor")
    collision_monitor_params = LaunchConfiguration("collision_monitor_params")
    verbose = LaunchConfiguration("verbose")
    world = LaunchConfiguration("world")

    robot_xacro = PathJoinSubstitution(
        [
            FindPackageShare("zhirong_description"),
            "urdf",
            "zhirong_diffbot.urdf.xacro",
        ]
    )
    robot_description = ParameterValue(
        Command(["xacro ", robot_xacro]),
        value_type=str,
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
            "verbose": verbose,
            "server_required": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_zhirong_diffbot",
        output="screen",
        arguments=[
            "-entity",
            "zhirong_diffbot",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.02",
        ],
    )

    spawn_qr_marker = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_nav_home_qr_marker",
        output="screen",
        arguments=[
            "-entity",
            "nav_home_qr_marker",
            "-file",
            PathJoinSubstitution(
                [
                    FindPackageShare("zhirong_gazebo"),
                    "models",
                    "zhirong_qr_marker",
                    "model.sdf",
                ]
            ),
            "-x",
            "2.82",
            "-y",
            "-1.75",
            "-z",
            "0.42",
            "-P",
            "-1.57079632679",
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
        condition=IfCondition(slam),
    )

    collision_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_collision_monitor"),
                    "launch",
                    "collision_monitor_node.launch.py",
                ]
            )
        ),
        launch_arguments={
            "params_file": collision_monitor_params,
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(safety_monitor),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="ALSOFT_DRIVERS",
                value="null",
            ),
            SetEnvironmentVariable(
                name="GAZEBO_MODEL_DATABASE_URI",
                value="/",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Launch the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz2 with the robot and laser configuration.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("zhirong_bringup"),
                        "rviz",
                        "zhirong_lidar.rviz",
                    ]
                ),
                description="Absolute path to the RViz2 configuration file.",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="false",
                description="Start online asynchronous SLAM from /scan and /odom.",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("zhirong_bringup"),
                        "config",
                        "slam_toolbox_mapping.yaml",
                    ]
                ),
                description="Absolute path to the slam_toolbox parameters file.",
            ),
            DeclareLaunchArgument(
                "safety_monitor",
                default_value="true",
                description="Filter /cmd_vel through the laser safety zones.",
            ),
            DeclareLaunchArgument(
                "collision_monitor_params",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("zhirong_bringup"),
                        "config",
                        "collision_monitor.yaml",
                    ]
                ),
                description="Collision Monitor parameter file.",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="false",
                description="Enable verbose Gazebo server output.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("zhirong_gazebo"),
                        "worlds",
                        "test_world.world",
                    ]
                ),
                description="Absolute path to a Gazebo world file.",
            ),
            gazebo_launch,
            robot_state_publisher,
            spawn_robot,
            TimerAction(period=2.0, actions=[spawn_qr_marker]),
            slam_toolbox_launch,
            collision_monitor_launch,
            rviz_node,
        ]
    )

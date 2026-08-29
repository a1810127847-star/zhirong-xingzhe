from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    autostart = LaunchConfiguration("autostart")
    gui = LaunchConfiguration("gui")
    map_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    navigation_rviz = LaunchConfiguration("navigation_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
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
            "rviz": "false",
            "slam": "false",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "localization_launch.py",
                ]
            )
        ),
        launch_arguments={
            "autostart": autostart,
            "map": map_file,
            "namespace": "",
            "params_file": nav2_params_file,
            "use_composition": "False",
            "use_respawn": "False",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    common_remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]
    guarded_bt_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key="nav2_raw",
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )
    guarded_through_poses_bt = PathJoinSubstitution(
        [
            FindPackageShare("zhirong_bringup"),
            "behavior_trees",
            "navigate_through_poses_guarded.xml",
        ]
    )
    guarded_to_pose_bt = PathJoinSubstitution(
        [
            FindPackageShare("zhirong_bringup"),
            "behavior_trees",
            "navigate_to_pose_safe_recovery.xml",
        ]
    )
    bt_remappings = [
        ("tf", "/tf"),
        ("tf_static", "/tf_static"),
        ("compute_path_to_pose", "/compute_path_to_pose"),
        (
            "compute_path_through_poses",
            "/compute_path_through_poses",
        ),
        ("follow_path", "/follow_path"),
        ("backup", "/backup"),
        ("spin", "/spin"),
        ("wait", "/wait"),
        (
            "global_costmap/clear_entirely_global_costmap",
            "/global_costmap/clear_entirely_global_costmap",
        ),
        (
            "local_costmap/clear_entirely_local_costmap",
            "/local_costmap/clear_entirely_local_costmap",
        ),
    ]
    navigation_nodes = GroupAction(
        [
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[nav2_params_file],
                arguments=["--ros-args", "--log-level", "info"],
                remappings=common_remappings
                + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="zhirong_bringup",
                executable="patrol_angular_guard.py",
                name="patrol_angular_guard",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[nav2_params_file],
                arguments=["--ros-args", "--log-level", "info"],
                remappings=common_remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params_file],
                arguments=["--ros-args", "--log-level", "info"],
                remappings=common_remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[nav2_params_file],
                arguments=["--ros-args", "--log-level", "info"],
                remappings=common_remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                namespace="nav2_raw",
                name="bt_navigator",
                output="screen",
                parameters=[
                    guarded_bt_params,
                    {
                        "default_nav_to_pose_bt_xml": guarded_to_pose_bt,
                        "default_nav_through_poses_bt_xml": (
                            guarded_through_poses_bt
                        )
                    },
                ],
                arguments=["--ros-args", "--log-level", "info"],
                remappings=bt_remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[nav2_params_file],
                arguments=["--ros-args", "--log-level", "info"],
                remappings=common_remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[nav2_params_file],
                arguments=["--ros-args", "--log-level", "info"],
                remappings=common_remappings
                + [
                    ("cmd_vel", "cmd_vel_nav"),
                    ("cmd_vel_smoothed", "cmd_vel_nav_smoothed"),
                ],
            ),
            # Start the lifecycle manager after every managed process has had
            # time to create its transition services. On resource-heavy GUI
            # starts this prevents velocity_smoother being left unconfigured.
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        output="screen",
                        arguments=["--ros-args", "--log-level", "info"],
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {"autostart": autostart},
                            {
                                "node_names": [
                                    "controller_server",
                                    "smoother_server",
                                    "planner_server",
                                    "behavior_server",
                                    "waypoint_follower",
                                    "velocity_smoother",
                                ]
                            },
                        ],
                    )
                ],
            ),
            TimerAction(
                # The through-poses BT validates its planner action server at
                # configure time. Give the root navigation lifecycle manager
                # enough time to configure the planner first on WSLg starts.
                period=5.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        namespace="nav2_raw",
                        name="lifecycle_manager_bt",
                        output="screen",
                        arguments=["--ros-args", "--log-level", "info"],
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {"autostart": autostart},
                            {"node_names": ["bt_navigator"]},
                        ],
                    )
                ],
            ),
        ]
    )

    goal_guard = Node(
        package="zhirong_bringup",
        executable="goal_guard.py",
        name="goal_guard",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "goal_clearance_m": 0.34,
                "costmap_timeout_sec": 2.0,
                "occupied_threshold": 100,
                "raw_action_name": "/nav2_raw/navigate_to_pose",
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(navigation_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically activate the Nav2 lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Launch the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("zhirong_bringup"),
                        "maps",
                        "zhirong_test_map.yaml",
                    ]
                ),
                description="Absolute path to the saved occupancy-grid map.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("zhirong_bringup"),
                        "config",
                        "nav2_params.yaml",
                    ]
                ),
                description="Absolute path to the Nav2 parameter file.",
            ),
            DeclareLaunchArgument(
                "navigation_rviz",
                default_value="true",
                description="Launch the Nav2 RViz2 interface.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("zhirong_bringup"),
                        "rviz",
                        "zhirong_navigation.rviz",
                    ]
                ),
                description="Absolute path to the navigation RViz2 configuration.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time.",
            ),
            simulation_launch,
            goal_guard,
            # Give Gazebo and the sensor bridge enough time to settle before the
            # Nav2 lifecycle transition.  This avoids intermittent DDS service
            # response timeouts on slower GUI starts.
            TimerAction(
                period=6.0,
                actions=[
                    localization_launch,
                    navigation_nodes,
                ],
            ),
            rviz_node,
        ]
    )

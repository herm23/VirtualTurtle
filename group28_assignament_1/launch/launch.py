from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_name = 'group28_assignament_1'

    # Launch the simulation
    assignment1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ir_launch'),
                'launch',
                'assignment_1.launch.py'
            ])
        ]),
        launch_arguments={ 'ros-args': '--log-level WARN' }.items()
    )

    # launch apriltag_ros with output disabled
    apriltag_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'launch',
                'launch_apriltag.yaml'
            ])
        ),
        launch_arguments={ 'ros-args': '--log-level ERROR' }.items()
    )

    # multithreaded container for CircleDetector
    circle_container = ComposableNodeContainer(
        name='circle_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='detection::CircleDetector',
                name='circle_detector'
            )
        ],
        output='screen'
    )

    # Single-threaded container for TagDetector
    tag_container = ComposableNodeContainer(
        name='tag_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='detection::TagDetector',
                name='tag_detector'
            )
        ],
        output='screen'
    )

    delayed_nodes = TimerAction(
        period=5.0,
        actions=[circle_container, tag_container]
    )

    return LaunchDescription([
        assignment1_launch,
        apriltag_launch,
        delayed_nodes
    ])

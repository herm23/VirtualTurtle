from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_name = 'group28_assignament_1'

    # simulation launch
    assignment1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ir_launch'),
                'launch',
                'assignment_1.launch.py'
            ])
        ])
    )

    # apriltag_ros launch with custom configuration
    apriltag_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'launch',
                'launch_apriltag.yaml'
            ])
        )
    )

    # CircleDetector (with tuned parameters)
    circle_container = ComposableNodeContainer(
        name='circle_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='detection::CircleDetector',
                name='circle_detector',
                parameters=[{
                    'wait_timeout': 10,
                    'scan_topic': '/scan',
                    'cluster_distance': 0.3,
                    'smart_cluster': False,
                    'min_points': 3,
                    'min_distance': 0.0,
                    'max_radius': 0.5,
                    'max_mae': 0.04
                }]
            )
        ],
        output='screen'
    )

    # TagDetector
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

    # non-composable navigation node
    navigation_node = Node(
        package='controller',
        executable='navigation_node',
        name='navigation_node',
        output='screen'
    )

    # Delay composable detection nodes like before
    delayed_nodes = TimerAction(
        period=7.0,
        actions=[circle_container, tag_container]
    )

    more_delayed_nodes = TimerAction(
        period=10.0,
        actions=[navigation_node]
    )

    return LaunchDescription([
        assignment1_launch,
        apriltag_launch,
        delayed_nodes,
        more_delayed_nodes
    ])

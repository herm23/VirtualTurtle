####################################################
# this file launches all the nodes developed by us #
# for detection and navigation purposes            #
####################################################

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_name = 'group28_assignament_1'

    # Multithreaded container for CircleDetector
    # and parameters set to best values
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
                    'wait_timeout': 5,
                    'scan_topic': '/scan',
                    'cluster_distance': 0.3,
                    'smart_cluster' : False,
                    'min_points': 3,
                    'min_distance': 0.0,
                    'max_radius': 0.5,
                    'max_mae': 0.04
                }]
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

    # non-composable navigation node
    navigation_node = Node(
        package=pkg_name,
        executable='navigation_node',
        name='navigation_node',
        output='screen'
    )

    # delayed 3 seconds to ensure transforms
    # are already being published
    delayed_nodes = TimerAction(
        period=3.0,
        actions=[navigation_node]
    )

    return LaunchDescription([
        circle_container,
        tag_container,
        delayed_nodes
    ])
#########################################################
# this file launches the simulation provided in ir_2526 #
# as well as the modified apriltag_ros node             #
#########################################################

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
        launch_arguments={'ros-args': '--log-level WARN'}.items()
    )

    # Launch apriltag_ros with output disabled
    apriltag_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'launch',
                'launch_apriltag.yaml'
            ])
        ),
        launch_arguments={'ros-args': '--log-level ERROR'}.items()
    )

    return LaunchDescription([
        assignment1_launch,
        apriltag_launch
    ])
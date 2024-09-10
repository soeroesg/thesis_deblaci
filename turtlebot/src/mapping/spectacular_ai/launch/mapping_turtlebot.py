from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    spectacularai_node = Node(
        package='spectacularai_depthai',
        executable='ros2_node',
        parameters=[
            { 'recordingFolder': LaunchConfiguration("recordingFolder") },
        ],
    )

    return [
        spectacularai_node
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("recordingFolder", default_value='')
        ] + [
            OpaqueFunction(function=launch_setup)
        ]
    )

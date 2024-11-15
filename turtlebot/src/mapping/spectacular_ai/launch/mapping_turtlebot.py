from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    spectacularai_node = Node(
        package="spectacularai_depthai",
        executable="ros2_node",
        parameters=[
            {"recording_folder": LaunchConfiguration("recording_folder")},
            {"do_recording_and_slam": LaunchConfiguration("do_recording_and_slam")},
            {"publish_tf": LaunchConfiguration("publish_tf")},
            {"publish_pointcloud": LaunchConfiguration("publish_pointcloud")},
        ],
    )

    return [spectacularai_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("recording_folder", default_value=""),
            DeclareLaunchArgument("do_recording_and_slam", default_value="false"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("publish_pointcloud", default_value="true"),
        ]
        + [OpaqueFunction(function=launch_setup)]
    )

import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # YAML file containing BT executor parameters
    bt_config = os.path.join(get_package_share_directory("drt_behavior"), "config", "sample_bt_executor.yaml")

    behavior_executor = Node(
        package="drt_behavior",
        executable="demo",
        output="both",
        parameters=[bt_config],
    )

    return [behavior_executor]


def declare_arguments():
    return []


def generate_launch_description():
    return LaunchDescription([*declare_arguments(), OpaqueFunction(function=launch_setup)])


if __name__ == "__main__":
    generate_launch_description()

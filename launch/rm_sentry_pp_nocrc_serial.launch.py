from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory("rm_sentry_pp_nocrc_serial")
    yaml_path = os.path.join(pkg, "config", "serial.yaml")

    return LaunchDescription([
        Node(
            package="rm_sentry_pp_nocrc_serial",
            executable="rm_sentry_pp_nocrc_serial_node",
            name="rm_sentry_pp_nocrc_serial",
            output="screen",
            parameters=[yaml_path],
        )
    ])

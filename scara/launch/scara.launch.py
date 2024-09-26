import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Load the URDF into a parameter
    share_dir = get_package_share_directory("simple_robots")
    urdf_path = os.path.join(share_dir, "scara", "scara.urdf")
    urdf = open(urdf_path).read()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="scara_state_publisher",
                parameters=[{"robot_description": urdf, "use_sim_time": True}],
            ),
            Node(
                package="simple_robots",
                executable="scara.py",
                name="scara",
            ),
            Node(
                package="rviz2",
                namespace="",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d"
                    + os.path.join(
                        share_dir,
                        "rviz2",
                        "standard.rviz",
                    )
                ],
            ),
        ]
    )

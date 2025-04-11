from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "python3",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("gz_ros2_control_demos"),
                            "launch",
                            "ackermann_drive_example_test_helper.py",
                        ]
                    ),
                ],
                output="screen",
            )
        ]
    )
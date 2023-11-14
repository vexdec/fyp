from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Create a LaunchDescription object to hold the collection of launch actions
    return LaunchDescription(
        [
            # Launch a node running the "turtlesim" package with the "turtlesim_node" executable
            Node(
                package="turtlesim",
                executable="turtlesim_node",
            ),
            # Launch a node running the "turtle_controller" package with the "turtle_sensor" executable
            Node(
                package="turtle_controller",
                executable="turtle_sensor",
            ),
            # Launch a node running the "turtle_controller" package with the "turtle_action_server" executable
            Node(
                package="turtle_controller",
                executable="turtle_action_server",
            ),
        ]
    )


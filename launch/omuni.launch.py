#it is for Foxy
# it can work on Foxy or later, but I feel xml file is better.

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='can_plugins2',
                    plugin='slcan_bridge::SlcanBridge',
                    name='slcan_bridge'),
                ComposableNode(
                    package='omuni_example',
                    plugin='omuni_controller::OmuniController',
                    name='controller'),
            ],
            output='both',
    )

    return launch.LaunchDescription([container])
import launch
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes

def generate_launch_description():
    return launch.LaunchDescription([
        # Avvia il simulatore turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='simulated_turtle'
        ),
        # Crea un contenitore per i componenti
        ComposableNodeContainer(
            name='navigation_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                LoadComposableNodes(
                    target_container='navigation_container',
                    composable_node_descriptions=[
                        launch_ros.descriptions.ComposableNode(
                            package='navigation_action_server',
                            plugin='nav_action_server_lib::NavActionServer',
                            name='nav_action_server'
                        ),
                        launch_ros.descriptions.ComposableNode(
                            package='navigation_action_server',
                            plugin='nav_action_client_lib::NavActionClient',
                            name='nav_action_client'
                        )
                    ]
                )
            ]
        )
    ])

import launch
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return launch.LaunchDescription([
        # Avvia il simulatore turtlesim in un nuovo terminale
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='simulated_turtle',
            prefix='terminator -x'
        ),
        # Spawns a second turtle (run in the background launch terminal)
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "{x: 8.0, y: 8.0, theta: 0.0, name: 'turtle2'}"],
            output='screen'
        ),
        # Avvia il server in un nuovo terminale
        Node(
            package='navigation_action_server',
            executable='nav_action_server_node',
            name='nav_action_server',
            prefix='terminator -x'
        ),
        # Avvia il client in un nuovo terminale (per interazione CLI)
        Node(
            package='navigation_action_server',
            executable='nav_action_client_node',
            name='nav_action_client',
            prefix='terminator -x'
        )
    ])

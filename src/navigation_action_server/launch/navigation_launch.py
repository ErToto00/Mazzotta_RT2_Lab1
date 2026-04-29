import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Builds and returns the launch configuration (LaunchDescription).
    Defines the nodes and processes that make up the application.
    """
    
    pkg_bme_gazebo_sensors = get_package_share_directory('bme_gazebo_sensors')

    return launch.LaunchDescription([
        # Launches the Gazebo environment, robot, bridge, and RViz from bme_gazebo_sensors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_bme_gazebo_sensors, 'launch', 'spawn_robot.launch.py')),
            launch_arguments={'world': 'my.sdf', 'use_sim_time': 'True'}.items()
        ),
        
        # Launches the server in a new terminal using component_container_mt for multi-threading
        Node(
            package='rclcpp_components',
            executable='component_container_mt',
            name='nav_action_server_container',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        Node(
            package='navigation_action_server',
            executable='nav_action_server_node',
            name='nav_action_server',
            parameters=[{'use_sim_time': True}],
            remap=[('__node', 'nav_action_server')],
            output='screen'
        ),
        
        # Bridge for robot TF and Odom
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/model/mogi_bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/mogi_bot/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
            ],
            remappings=[
                ('/model/mogi_bot/tf', '/tf'),
                ('/model/mogi_bot/odom', '/odom')
            ],
            output='screen'
        ),
        
        # Launches the client in a new terminal (for CLI interaction)
        Node(
            package='navigation_action_server',
            executable='nav_action_client_node',
            name='nav_action_client',
            parameters=[{'use_sim_time': True}],
            prefix='xterm -e'
        )
    ])

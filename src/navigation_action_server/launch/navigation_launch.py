import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_bme_gazebo_sensors = get_package_share_directory('bme_gazebo_sensors')

    return launch.LaunchDescription([
        # Launches Gazebo and the robot from the bme_gazebo_sensors package[cite: 1]
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_bme_gazebo_sensors, 'launch', 'spawn_robot.launch.py')),
            launch_arguments={'world': 'my.sdf', 'use_sim_time': 'True'}.items()
        ),
        
        # Server Node: Added '-hold' to keep the window open after a crash[cite: 1]
        Node(
            package='navigation_action_server',
            executable='nav_action_server_node',
            name='nav_action_server',
            parameters=[{'use_sim_time': True}],
            remappings=[('__node', 'nav_action_server')],
            prefix='xterm -hold -e',
            output='screen'
        ),
        
        # FIXED Bridge: Added the /pose argument to bridge world-to-robot transforms[cite: 1]
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/model/mogi_bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/mogi_bot/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/mogi_bot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            remappings=[
                ('/model/mogi_bot/tf', '/tf'),
                ('/model/mogi_bot/odom', '/odom'),
                ('/model/mogi_bot/pose', '/tf')
            ],
            output='screen'
        ),
        
        # Client Node[cite: 1]
        Node(
            package='navigation_action_server',
            executable='nav_action_client_node',
            name='nav_action_client',
            parameters=[{'use_sim_time': True}],
            prefix='xterm -hold -e'
        )
    ])
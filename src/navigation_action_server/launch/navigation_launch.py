import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Builds and returns the launch configuration (LaunchDescription).
    Defines the nodes and processes that make up the application, launching
    each program in a dedicated terminal (via 'terminator -x').
    """
    
    # Path to the local URDF file
    pkg_share_dir = get_package_share_directory('navigation_action_server')
    urdf_file_path = os.path.join(pkg_share_dir, 'urdf', 'my_robot.urdf')
    
    # Read the URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()
        
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    return launch.LaunchDescription([
        # Launches the Gazebo Sim simulator with an empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': 'empty.sdf -r'}.items()
        ),
        
        # Publishes the state (TFs) of the robot using the URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),
        
        # Spawns the URDF entity inside Gazebo Sim
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_entity',
            arguments=['-name', 'my_robot', '-string', robot_desc, '-z', '0.05'],
            output='screen'
        ),
        
        # Bridge ROS 2 and Gazebo Transport topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            output='screen'
        ),
        
        # Launches RViz2 in a new terminal
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            prefix='terminator -x'
        ),
        
        # Launches the server in a new terminal
        Node(
            package='navigation_action_server',
            executable='nav_action_server_node',
            name='nav_action_server',
            parameters=[{'use_sim_time': True}],
            prefix='terminator -x'
        ),
        
        # Launches the client in a new terminal (for CLI interaction)
        Node(
            package='navigation_action_server',
            executable='nav_action_client_node',
            name='nav_action_client',
            parameters=[{'use_sim_time': True}],
            prefix='terminator -x'
        )
    ])

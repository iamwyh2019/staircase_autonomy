import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import yaml

def generate_launch_description():
    """
    Generates the launch description for the staircase robot node.

    """
    
    # Get the package share directory for 'staircase_perception' pkg to find config files
    pkg_share = get_package_share_directory('staircase_perception')
    
    # Define the default path to the parameter file
    unified_estimation_config_file = os.path.join(pkg_share, 'config', 'unified_estimation_config.yaml')

    # === 1. Declare Launch Arguments ===
    declare_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value="robot",
        description='Namespace to prefix robot sensor data topics'
    )
    
    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=unified_estimation_config_file,
        description='Full path to the robot configuration file.'
    )
    
    declare_launch_marker_publisher_arg = DeclareLaunchArgument(
        'launch_marker_publisher',
        default_value='false',
        description='Whether to launch the marker publisher node.'
    )
    
    declare_simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Use simulation clock source'
    )

    # === 2. Create Node Actions ===

    # Node for the main staircase detector
    staircase_estimation_robot_node = Node(
        package='staircase_perception',
        executable='staircase_estimation_robot_node',
        name='staircase_estimation_robot_node', 
        output='screen',
        emulate_tty=True,
        # The 'parameters' field takes a list of file paths. Here we use the value from the launch argument.
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'robot_topics_prefix': LaunchConfiguration('robot_namespace'),
                'use_sim_time': LaunchConfiguration('simulation'),
            },
        ]
    )
    simple_clutter_segmentation_node = Node(
        package='staircase_perception',
        executable='simple_clutter_segmentation_node',
        name='simple_clutter_segmentation_node', 
        output='screen',
        emulate_tty=True,
        # The 'parameters' field takes a list of file paths. Here we use the value from the launch argument.
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'robot_topics_prefix': LaunchConfiguration('robot_namespace'),
                'use_sim_time': LaunchConfiguration('simulation'),
            },
        ]
    )
    
    # Conditional node for the marker publisher.
    # This node will only be launched if the 'launch_marker_publisher' argument is set to 'true'.
    stair_viz_node = Node(
        package='staircase_perception',
        executable='staircase_marker_publisher.py',
        name='stair_viz_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'robot_topics_prefix': LaunchConfiguration('robot_namespace'),
                'use_sim_time': LaunchConfiguration('simulation'),
            },
        ],
        condition=IfCondition(LaunchConfiguration('launch_marker_publisher'))
    )

    # === 3. Create and Return the LaunchDescription ===
    ld = LaunchDescription()
    
    # Add the declared arguments to the launch description
    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_config_file_arg)
    ld.add_action(declare_launch_marker_publisher_arg)
    ld.add_action(declare_simulation_arg)
    
    # Add the group of nodes to the launch description
    ld.add_action(staircase_estimation_robot_node)
    ld.add_action(simple_clutter_segmentation_node)
    ld.add_action(stair_viz_node)

    return ld


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    """
    Generates the launch description for the staircase robot node.

    """
    
    # Get the package share directory for 'staircase_perception' pkg to find config files
    pkg_share = get_package_share_directory('staircase_perception')
    
    # Define the default path to the parameter file
    unified_estimation_config_file = os.path.join(pkg_share, 'config', 'unified_estimation_config.yaml')

    # === 1. Declare Launch Arguments ==
    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=unified_estimation_config_file,
        description='Full path to the robot configuration file.'
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
        executable='staircase_client_node',
        name='staircase_client_node', 
        output='screen',
        emulate_tty=True,
        # The 'parameters' field takes a list of file paths. Here we use the value from the launch argument.
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('simulation'),
            },
        ]
    )

    # === 3. Create and Add Launch Descriptions ===
    ld = LaunchDescription()
    
    ld.add_action(declare_config_file_arg)
    ld.add_action(declare_simulation_arg)
    
    ld.add_action(staircase_estimation_robot_node)

    return ld


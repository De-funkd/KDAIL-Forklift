import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launches all nodes for the UWB visualization."""
    
    pkg_share = get_package_share_directory('Forklift_Firmware')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'uwb_visualization.rviz')

    # Static transform from world to the UWB base frame
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_uwb_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'uwb_base'],
        output='screen'
    )

    # UWB log parser node
    uwb_log_parser_node = Node(
        package='Forklift_Firmware',
        executable='uwb_data_parser.py',
        name='uwb_log_parser',
        output='screen'
    )

    # UWB position calculator node
    uwb_position_calculator_node = Node(
        package='Forklift_Firmware',
        executable='uwb_xy.py',
        name='uwb_position_calculator',
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        static_transform_publisher,
        uwb_log_parser_node,
        uwb_position_calculator_node,
        rviz_node
    ])

# modbus_node_launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    modbus_config_path = get_package_share_directory('mduino') + '/config/modbus_config.yaml'
    print(f"Modbus Config Path: {modbus_config_path}")

    config = os.path.join(
        get_package_share_directory('mduino'),
        'config',
        'modbus_config.yaml'
        )
    modbus_node = Node(
        package='mduino',  # Replace with your actual package name
        executable='mduino_modbus_node.py',
        name='mduino_modbus_node',
        output='screen',
        parameters=[config],
        remappings=[('/mduino_query', '/mduino_query'), ('/mduino_response', '/mduino_response'), ('/mduino_state', '/mduino_state')],
    )

    return LaunchDescription([modbus_node])

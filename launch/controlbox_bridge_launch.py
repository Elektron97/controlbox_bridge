from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
    
def generate_launch_description():
    
    config_file_serial = os.path.join(
        get_package_share_directory('controlbox_bridge'),
        'config',
        'serial_com.yaml'
    )

    config_file_hardware = os.path.join(
        get_package_share_directory('controlbox_bridge'),
        'config',
        'hardware_params_9chambers.yaml'
    )

    config_file_params = os.path.join(
        get_package_share_directory('controlbox_bridge'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='controlbox_bridge',
            executable='controlbox_bridge_node',
            name='controlbox_bridge_node',
            output='screen',
            parameters=[config_file_params]
            # parameters=[config_file_serial, config_file_hardware]
        ),

##### Uncomment to start RQT automatically when launching this file
        # # Start RQT
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'rqt_gui', 'rqt_gui'],
        #     output='screen'
        # )
    ])
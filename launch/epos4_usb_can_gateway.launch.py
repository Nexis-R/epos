import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('epos'),
        'config',
        'epos4_usb_can_gateway.yaml'
        )
        
    node=Node(
        package = 'epos',
        name = 'epos',
        executable = 'epos',
        output='screen',
        parameters = [config]
    )

    ld.add_action(node)
    return ld
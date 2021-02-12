import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('robot_interface_eki'),
        'config',
        'params.yaml'
    )

  
    
    return LaunchDescription([
        Node(
            package='robot_interface_eki',
            node_executable='Server',
            node_name='Server',
            parameters=[config],
            output='screen'
        ),

        #Node(
        #    package='robot_interface_eki',
        #    node_executable='client.py',
        #    node_name='client',
        #    output='screen'
        #)


    ])

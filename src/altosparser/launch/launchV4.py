
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('altosparser')
    v4_parser = Node(
        package='altosparser',
        executable='altosparser',
        name='altosparser',
        namespace='v4',
        parameters=[os.path.join(pkg_share, 'param', 'altosParserV4.yaml')],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        v4_parser
    ])
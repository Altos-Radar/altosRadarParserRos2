
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('altosparser')
    rcu_parser = Node(
        package='altosparser',
        executable='altosparser',
        name='altosparser',
        namespace='rcu',
        parameters=[os.path.join(pkg_share, 'param', 'altosParserRcu.yaml')],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        rcu_parser
    ])
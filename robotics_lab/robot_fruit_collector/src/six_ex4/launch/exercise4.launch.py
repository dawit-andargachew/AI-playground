from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the original exercise_4 launch file from ir_2526
    ir_launch_dir = get_package_share_directory('ir_launch')
    exercise4_launch = os.path.join(ir_launch_dir, 'exercise_4.launch.py')
    
    return LaunchDescription([
        # Include the original initialization launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(exercise4_launch),
        ),
        
        # Launch turtlebot node (service server)
        Node(
            package='six_ex4',
            executable='turtlebot_node',
            name='turtlebot_node',
            output='screen',
            emulate_tty=True,
        ),
        
        # Launch burrow node (service client) with delay
        Node(
            package='six_ex4',
            executable='burrow_node',
            name='burrow_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
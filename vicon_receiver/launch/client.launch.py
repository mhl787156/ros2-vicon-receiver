from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    hostname = os.getenv("VICON_HOST_IP", default='127.0.0.1')
    buffer_size = os.getenv("VICON_BUFFER_SIZE", 200)
    topic_namespace = os.getenv("VICON_NAMEPSACE", default='vicon')

    return LaunchDescription([Node(
            package='vicon_receiver', executable='vicon_client', output='screen',
            parameters=[{
                'hostname': hostname, 
                'buffer_size': buffer_size, 
                'namespace': topic_namespace
            }]
        )])

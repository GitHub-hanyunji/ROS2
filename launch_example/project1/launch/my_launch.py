from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project1',
            namespace='psub1',
            executable='pub',
            name='node_pub1',
        ),
        Node(
            package='project1',
            namespace='psub1',
            executable='sub',
            name='node_sub1',
        ),
        Node(
            package='project1',
            namespace='psub2',
            executable='pub',
            name='node_pub1',
        ),
        Node(
            package='project1',
            namespace='psub2',
            executable='sub',
            name='node_sub1',
        )
    ])
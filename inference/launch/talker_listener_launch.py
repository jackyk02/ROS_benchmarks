from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talking',
            namespace='ns1',
            executable='talker',
            name='talker'
        ),
        Node(
            package='talking',
            namespace='ns2',
            executable='listener',
            name='listener'
        )
    ])

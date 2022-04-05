from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='opencv_demos',
            executable='face_detector',
            name='face_detector',
            remappings=[
                ('in_image_base_topic', '/image_raw'),
                ('out_image_base_topic', '/face_detector/out_image_base_topic'),
            ]
        )
    ])
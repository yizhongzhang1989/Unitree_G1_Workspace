from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera 100 - Low latency settings
        Node(
            package='camera_node',
            executable='camera_node',
            name='camera_node100',
            output='screen',
            parameters=[{
                'camera_name': 'camera100',
                'rtsp_url_main': 'rtsp://admin:123456@192.168.1.100/stream0',
                'camera_ip': '192.168.1.100',
                'server_port': 8010,
                'stream_fps': 10,  # Lower FPS for less latency
                'jpeg_quality': 40,  # Lower quality for faster encoding
                'max_width': 480,  # Smaller resolution for minimal latency
                'publish_ros_image': False,
                'ros_topic_name': '/robot_camera100/image_raw'
            }]
        ),
        # Camera 101 - Even lower settings for stability
        Node(
            package='camera_node',
            executable='camera_node',
            name='camera_node101',
            output='screen',
            parameters=[{
                'camera_name': 'camera101',
                'rtsp_url_main': 'rtsp://admin:123456@192.168.1.101/stream0',
                'camera_ip': '192.168.1.101',
                'server_port': 8011,
                'stream_fps': 10,  # Even lower FPS
                'jpeg_quality': 40,  # Lower quality
                'max_width': 480,  # Even smaller for minimal latency
                'publish_ros_image': False,
                'ros_topic_name': '/robot_camera101/image_raw'
            }]
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Single Camera Stream with Web Visualization
        Node(
            package='camera_node',
            executable='camera_node',
            name='camera_node_stream102',
            output='screen',
            parameters=[{
                'camera_name': 'stream102',
                'rtsp_url_main': 'rtsp://admin:123456@192.168.1.102/stream0',
                'camera_ip': '192.168.1.102',
                'server_port': 8081,
                'stream_fps': 30,
                'jpeg_quality': 60,
                'max_width': 640,
                'publish_ros_image': True,
                'ros_topic_name': '/robot_arm_camera102/image_raw'
            }]
        ),
    ])

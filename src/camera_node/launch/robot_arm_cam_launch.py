from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Declare launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='RobotArmCamera',
        description='Name of the camera'
    )
    
    rtsp_url_main_arg = DeclareLaunchArgument(
        'rtsp_url_main',
        default_value='rtsp://admin:123456@192.168.1.102/stream0',
        description='RTSP URL for main stream (1080p)'
    )
    
    camera_ip_arg = DeclareLaunchArgument(
        'camera_ip',
        default_value='192.168.1.102',
        description='IP address of the camera'
    )
    
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='8012',
        description='Port for Flask web server'
    )
    
    stream_fps_arg = DeclareLaunchArgument(
        'stream_fps',
        default_value='25',
        description='Target FPS for video streaming'
    )
    
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='75',
        description='JPEG compression quality (1-100)'
    )
    
    max_width_arg = DeclareLaunchArgument(
        'max_width',
        default_value='800',
        description='Maximum width for web streaming (pixels)'
    )
    
    publish_ros_image_arg = DeclareLaunchArgument(
        'publish_ros_image',
        default_value='true',
        description='Whether to publish ROS2 Image messages'
    )
    
    ros_topic_name_arg = DeclareLaunchArgument(
        'ros_topic_name',
        default_value='/robot_arm_camera/image_raw',
        description='ROS2 topic name for image publishing'
    )
    
    # Create the camera node
    camera_node = Node(
        package='camera_node',
        executable='camera_node',
        name='camera_node',
        parameters=[
            {'camera_name': LaunchConfiguration('camera_name')},
            {'rtsp_url_main': LaunchConfiguration('rtsp_url_main')},
            {'camera_ip': LaunchConfiguration('camera_ip')},
            {'server_port': LaunchConfiguration('server_port')},
            {'stream_fps': LaunchConfiguration('stream_fps')},
            {'jpeg_quality': LaunchConfiguration('jpeg_quality')},
            {'max_width': LaunchConfiguration('max_width')},
            {'publish_ros_image': LaunchConfiguration('publish_ros_image')},
            {'ros_topic_name': LaunchConfiguration('ros_topic_name')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        camera_name_arg,
        rtsp_url_main_arg,
        camera_ip_arg,
        server_port_arg,
        stream_fps_arg,
        jpeg_quality_arg,
        max_width_arg,
        publish_ros_image_arg,
        ros_topic_name_arg,
        camera_node
    ])

# Camera Node

A generic ROS2 node for IP camera RTSP streaming and control with comprehensive web interface and flexible configuration options.

## Features

- **RTSP Stream Processing**: High-performance FFmpeg-based RTSP stream handling with error recovery
- **Web-based Control Interface**: Full-featured web interface with real-time controls and monitoring
- **Event-driven ROS2 Publishing**: Optimized ROS2 Image topic publishing with frame synchronization
- **Camera Services**: Snapshot capture and node restart services
- **Multi-resolution Support**: Dynamic switching between main/sub streams (1920x1080 / 640x480)
- **Performance Tuning**: Configurable FPS, JPEG quality, and stream resolution
- **Viewer Tracking**: Optimized streaming for multiple concurrent web viewers
- **Error Handling**: Robust error recovery with automatic reconnection attempts
- **Background Process Support**: Non-blocking operations with proper resource management

## Dependencies

### ROS2 Requirements
- ROS2 (tested with Humble)
- Python packages:
  - `rclpy` - ROS2 Python client library
  - `sensor_msgs` - ROS2 sensor message types
  - `std_srvs` - ROS2 standard service types
  - `cv_bridge` - OpenCV-ROS2 bridge

### Python Dependencies
- `opencv-python` - Computer vision library
- `numpy` - Numerical computing library
- `flask` - Web framework for control interface

### System Dependencies
- `ffmpeg` - Video processing (with libx264 support)
- `ffprobe` - Media stream analysis tool

### Optional Tools
- `ffplay` - For manual RTSP stream testing

## Installation

1. **Clone to ROS2 workspace** (if not already present):
```bash
cd ~/your_workspace/src
# Package should already be here as part of the robot_dc project
```

2. **Install system dependencies**:
```bash
sudo apt update
sudo apt install ffmpeg ffprobe
```

3. **Install Python dependencies**:
```bash
pip3 install flask opencv-python numpy
# Or use requirements.txt if available:
# pip3 install -r requirements.txt
```

4. **Build the package**:
```bash
cd ~/your_workspace
colcon build --packages-select camera_node
```

5. **Source the workspace**:
```bash
source install/setup.bash
```

## Quick Start

### Launch with Default Configuration

```bash
ros2 run camera_node camera_node
```

### Launch with Launch File

```bash
# Single camera launch
ros2 launch camera_node single_stream_test.py

# Multiple cameras launch  
ros2 launch camera_node double_camera_launch.py

# Robot arm camera launch
ros2 launch camera_node robot_arm_cam_launch.py
```

### Launch with Custom Parameters

```bash
ros2 run camera_node camera_node \
    --ros-args \
    -p camera_name:=MyCam \
    -p rtsp_url_main:=rtsp://admin:password@192.168.1.100/stream0 \
    -p camera_ip:=192.168.1.100 \
    -p server_port:=8010 \
    -p ros_topic_name:=/my_camera/image_raw \
    -p publish_ros_image:=true
```

### Available ROS2 Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_name` | string | `'GenericCamera'` | Camera identifier for services and logging |
| `rtsp_url_main` | string | `'rtsp://admin:123456@192.168.1.100/stream0'` | RTSP stream URL |
| `camera_ip` | string | `'192.168.1.100'` | Camera IP address for reference |
| `server_port` | int | `8010` | Web interface port |
| `stream_fps` | int | `25` | Target streaming FPS for web interface |
| `jpeg_quality` | int | `75` | JPEG compression quality (1-100) |
| `max_width` | int | `800` | Maximum streaming width in pixels |
| `publish_ros_image` | bool | `true` | Enable/disable ROS2 image publishing |
| `ros_topic_name` | string | `'/camera/image_raw'` | ROS2 image topic name |

## Web Interface

Access the comprehensive web interface at: `http://localhost:{server_port}` (default: http://localhost:8010)

### Web Interface Features

- **📹 Live Camera Preview**: Real-time MJPEG streaming with automatic error recovery
- **🎮 Camera Controls**: 
  - ▶️ Start/⏹️ Stop camera
  - 🔄 Restart camera with full reinitialization
  - 📸 Take and download snapshots
- **📺 Stream Quality Control**: Dynamic switching between resolutions
  - **Main Stream**: 1920x1080 (high quality)
  - **Sub Stream**: 640x480 (low bandwidth)
- **🤖 ROS2 Integration Controls**:
  - Toggle ROS2 image publishing on/off
  - Real-time status monitoring
- **📊 System Information Panel**:
  - Node configuration details
  - ROS2 topic and service information
  - Network configuration
- **📱 Responsive Design**: Mobile-friendly interface with modern styling

### Web API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Main web interface |
| `/video_feed` | GET | MJPEG video stream |
| `/status` | GET | Camera status information |
| `/start` | POST | Start camera stream |
| `/stop` | POST | Stop camera stream |
| `/restart` | POST | Restart camera stream |
| `/snapshot` | POST | Take snapshot |
| `/download_snapshot` | GET | Download latest snapshot |
| `/ros_image_publish` | GET/POST | Get/toggle ROS2 publishing status |

## ROS2 Integration

### Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/{camera_name}/image_raw` | `sensor_msgs/Image` | Camera image stream (event-driven publishing) |

**Note**: The `camera_name` parameter determines the topic namespace. Default topic is `/GenericCamera/image_raw`.

### Services

| Service | Service Type | Description |
|---------|--------------|-------------|
| `/{camera_name}/take_snapshot` | `std_srvs/Trigger` | Capture and save camera snapshot to `/tmp/{camera_name}_snapshots/` |
| `/restart_{camera_name}_node` | `std_srvs/Trigger` | Restart camera node with full reinitialization |

### Event-Driven Publishing

The node uses **event-driven ROS2 image publishing** instead of timer-based publishing for optimal performance:
- Images are published only when new frames are available
- Eliminates unnecessary CPU usage from empty publishes
- Provides frame synchronization between web streaming and ROS2 publishing
- Can be toggled on/off during runtime via web interface or service calls

## Configuration Examples

### Multiple Camera Setup

Run multiple camera nodes with different configurations:

```bash
# Camera 1 - High quality main camera
ros2 run camera_node camera_node \
    --ros-args \
    -p camera_name:=Camera1 \
    -p rtsp_url_main:=rtsp://admin:123456@192.168.1.100/stream0 \
    -p server_port:=8010 \
    -p ros_topic_name:=/camera1/image_raw \
    -p publish_ros_image:=true

# Camera 2 - Secondary camera
ros2 run camera_node camera_node \
    --ros-args \
    -p camera_name:=Camera2 \
    -p rtsp_url_main:=rtsp://admin:123456@192.168.1.101/stream0 \
    -p server_port:=8011 \
    -p ros_topic_name:=/camera2/image_raw \
    -p publish_ros_image:=true
```

### Performance Optimization

**High-Performance Configuration** (for powerful systems):
```bash
ros2 run camera_node camera_node \
    --ros-args \
    -p stream_fps:=30 \
    -p jpeg_quality:=90 \
    -p max_width:=1280 \
    -p publish_ros_image:=true
```

**Low-Bandwidth Configuration** (for network-constrained environments):
```bash
ros2 run camera_node camera_node \
    --ros-args \
    -p stream_fps:=15 \
    -p jpeg_quality:=50 \
    -p max_width:=640 \
    -p publish_ros_image:=false
```

**Web-Only Configuration** (disable ROS2 publishing for web streaming only):
```bash
ros2 run camera_node camera_node \
    --ros-args \
    -p publish_ros_image:=false \
    -p server_port:=8080
```

## Troubleshooting

### Camera Connection Issues

1. **Test RTSP URL accessibility**:
```bash
# Test with ffplay (install with: sudo apt install ffmpeg)
ffplay rtsp://admin:password@192.168.1.100/stream0

# Test with ffprobe for stream information
ffprobe -rtsp_transport tcp rtsp://admin:password@192.168.1.100/stream0
```

2. **Check network connectivity**:
```bash
ping 192.168.1.100
telnet 192.168.1.100 554  # RTSP default port
```

3. **Verify camera credentials and stream paths**:
   - Check camera documentation for correct RTSP URLs
   - Verify username/password combination
   - Test different stream paths (`/stream0`, `/stream1`, `/h264`, etc.)

### Performance Issues

1. **Reduce streaming parameters**:
   - Lower `stream_fps` (try 15-20 FPS)
   - Reduce `jpeg_quality` (try 50-60)
   - Decrease `max_width` (try 640 or 480)

2. **Switch to sub stream**:
   - Use web interface to switch from main to sub stream
   - Sub stream typically uses less bandwidth

3. **Disable unnecessary features**:
   - Set `publish_ros_image:=false` if ROS2 publishing not needed
   - This reduces CPU usage significantly

4. **Check system resources**:
```bash
# Monitor CPU and memory usage
htop

# Check network bandwidth
iftop -i your_network_interface
```

### FFmpeg Issues

1. **FFmpeg process fails to start**:
   - Check FFmpeg installation: `ffmpeg -version`
   - Verify camera supports TCP transport
   - Try different RTSP transport options

2. **Stream resolution detection fails**:
   - Node will fall back to default 1920x1080
   - Manually verify with: `ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of json YOUR_RTSP_URL`

### ROS2 Integration Issues

1. **Image topic not publishing**:
   - Check if `publish_ros_image` parameter is true
   - Verify topic with: `ros2 topic list | grep image`
   - Monitor topic: `ros2 topic echo /{camera_name}/image_raw --no-arr`

2. **Service calls failing**:
   - List available services: `ros2 service list | grep camera`
   - Test service: `ros2 service call /{camera_name}/take_snapshot std_srvs/srv/Trigger`

### Build Issues

1. **Missing dependencies**:
```bash
# Install missing ROS2 packages
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs

# Install Python packages
pip3 install flask opencv-python numpy
```

2. **Package build fails**:
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --packages-select camera_node --cmake-clean-cache
```

### Web Interface Issues

1. **Cannot access web interface**:
   - Check if port is available: `netstat -tlnp | grep :8010`
   - Try different port in parameters
   - Check firewall settings

2. **Video feed shows error**:
   - Camera stream may not be running
   - Check browser console for JavaScript errors
   - Try refreshing the page

## Architecture

### Class Structure

- **`CameraNode`**: Main ROS2 node class
  - Handles ROS2 parameter management
  - Manages Flask web server
  - Provides ROS2 services for snapshot and restart
  - Coordinates between web interface and ROS2 publishing

- **`RTSPStream`**: RTSP stream handling class
  - FFmpeg process management with error recovery
  - Frame processing and thread-safe access
  - Event-driven ROS2 image publishing
  - Multi-viewer web streaming optimization
  - Automatic reconnection on stream failures

### Threading Model

- **Main Thread**: ROS2 node execution and service handling
- **Flask Thread**: Web server and HTTP request handling  
- **Stream Thread**: FFmpeg frame reading and processing
- **ROS Publishing Thread**: Event-driven image message publishing

### Error Recovery

- **Stream Failures**: Automatic reconnection with exponential backoff
- **FFmpeg Crashes**: Process restart with configurable retry attempts
- **Network Issues**: Graceful fallback to error images
- **Resource Cleanup**: Proper thread and process termination

## Advanced Usage

### Custom Stream URLs

The node supports various RTSP URL formats:
```bash
# Standard format
rtsp://username:password@ip:port/path

# Examples for common camera brands
rtsp://admin:123456@192.168.1.100/stream0    # Generic IP camera
rtsp://admin:123456@192.168.1.100/h264       # H.264 stream
rtsp://admin:123456@192.168.1.100/mjpeg      # MJPEG stream
```

### Integration with Robot Systems

```bash
# Integration with robot arm camera
ros2 launch camera_node robot_arm_cam_launch.py

# Multiple camera setup for robot monitoring
ros2 launch camera_node double_camera_launch.py
```

### Programmatic Control

```python
# Python example for taking snapshots
import rclpy
from std_srvs.srv import Trigger

# Initialize ROS2
rclpy.init()
node = rclpy.create_node('camera_client')

# Create service client
client = node.create_client(Trigger, '/GenericCamera/take_snapshot')
client.wait_for_service()

# Take snapshot
request = Trigger.Request()
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)

if future.result().success:
    print(f"Snapshot taken: {future.result().message}")
else:
    print(f"Failed: {future.result().message}")
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes with appropriate tests
4. Submit a pull request

### Development Guidelines

- Follow ROS2 Python coding standards
- Add appropriate logging for debugging
- Handle exceptions gracefully
- Update documentation for new features
- Test with multiple camera types and network conditions

## License

This project is part of the robot_dc robotics system.

---

**For more information about the robot_dc project, see the main repository documentation.**

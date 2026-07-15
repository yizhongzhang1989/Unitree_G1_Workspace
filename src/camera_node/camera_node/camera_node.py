import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
import subprocess
import threading
import numpy as np
import cv2
import time
import os
import json
from datetime import datetime
from flask import Flask, Response, jsonify, request
from cv_bridge import CvBridge


def get_stream_resolution(url, max_retries=3, retry_delay=2):
    "get the resolution of the RTSP stream using ffprobe with retries"
    for attempt in range(max_retries):
        try:
            cmd = [
                "ffprobe",
                "-v", "error",
                "-select_streams", "v:0",
                "-show_entries", "stream=width,height",
                "-of", "json",
                url
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode != 0:
                print(f"ffprobe failed for {url}, attempt {attempt + 1}/{max_retries}: {result.stderr}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    print(f"Using default resolution for {url}")
                    return 1920, 1080
            
            info = json.loads(result.stdout)
            
            if "streams" not in info or len(info["streams"]) == 0:
                print(f"No streams found for {url}, attempt {attempt + 1}/{max_retries}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    print(f"Using default resolution for {url}")
                    return 1920, 1080
                    
            stream = info["streams"][0]
            if 'width' not in stream or 'height' not in stream:
                print(f"No resolution info found for {url}, attempt {attempt + 1}/{max_retries}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    print(f"Using default resolution for {url}")
                    return 1920, 1080
                    
            w = stream["width"]
            h = stream["height"]
            return w, h
            
        except (json.JSONDecodeError, KeyError, subprocess.TimeoutExpired) as e:
            print(f"Error getting resolution for {url}, attempt {attempt + 1}/{max_retries}: {e}")
            if attempt < max_retries - 1:
                time.sleep(retry_delay)
                continue
    
    # This should never be reached, but just in case
    return 1920, 1080


class RTSPStream:
    """RTSP stream handler using ffmpeg."""
    
    def __init__(self, name, url, node=None):
        self.name = name
        self.url = url
        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        self.node = node  # ROS2 node reference for logging
        
        # Add viewer tracking for streaming
        self.viewer_count = 0
        self.viewer_lock = threading.Lock()
        
        # Event-driven ROS publishing
        self.ros_publish_enabled = False
        self.ros_publisher = None
        self.cv_bridge = None
        self.last_published_frame_id = -1
        self.frame_ready_event = threading.Event()
        
        self.log_info(f"Getting resolution for {name} stream: {url}")
        self.width, self.height = get_stream_resolution(self.url)
        self.log_info(f"Resolution for {name}: {self.width}x{self.height}")
        
        # Add frame tracking for reconnection detection
        self.frame_count = 0
        self.last_frame_update = time.time()
        
        self.log_info(f"Starting FFmpeg process for {name}...")
        # Add retry mechanism for FFmpeg process startup
        max_ffmpeg_retries = 3
        self.proc = None
        self.available = False
        
        for attempt in range(max_ffmpeg_retries):
            try:
                self.proc = subprocess.Popen(
                    [
                        "ffmpeg",
                        "-rtsp_transport", "tcp",
                        "-fflags", "nobuffer",
                        "-flags", "low_delay",
                        "-an",
                        "-i", self.url,
                        "-f", "rawvideo",
                        "-pix_fmt", "bgr24",
                        "-"
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL
                )
                # Give FFmpeg a moment to start and establish connection
                time.sleep(3)  # Increased wait time for connection establishment
                if self.proc.poll() is None:  # Process is still running
                    self.available = True
                    self.log_info(f"FFmpeg process started for {name}, stream will be validated during frame reading")
                    break
                else:
                    self.log_warn(f"FFmpeg process failed for {name}, attempt {attempt + 1}/{max_ffmpeg_retries}")
                    if attempt < max_ffmpeg_retries - 1:
                        time.sleep(2)
                        continue
            except Exception as e:
                self.log_error(f"Error starting FFmpeg for {name}, attempt {attempt + 1}/{max_ffmpeg_retries}: {e}")
                if attempt < max_ffmpeg_retries - 1:
                    time.sleep(2)
                    continue
        
        if not self.available:
            self.log_error(f"Failed to start FFmpeg for {name} after {max_ffmpeg_retries} attempts. Stream will be unavailable.")
            self.proc = None
        
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()
        
        if self.available:
            self.log_info(f"Started {name} stream successfully")
        else:
            self.log_warn(f"Stream {name} is not available - will show error message")

    def log_info(self, message):
        """Log info message using ROS2 node if available"""
        if self.node:
            self.node.get_logger().info(message)
        else:
            print(f"[INFO] {message}")

    def log_warn(self, message):
        """Log warning message using ROS2 node if available"""
        if self.node:
            self.node.get_logger().warn(message)
        else:
            print(f"[WARN] {message}")

    def log_error(self, message):
        """Log error message using ROS2 node if available"""
        if self.node:
            self.node.get_logger().error(message)
        else:
            print(f"[ERROR] {message}")

    def update(self):
        """Continuously read frames from FFmpeg process"""
        if not self.available or self.proc is None:
            return
            
        frame_size = self.width * self.height * 3
        consecutive_failures = 0
        max_consecutive_failures = 10
        
        while self.running and self.available:
            try:
                if self.proc.stdout is None:
                    self.log_error(f"FFmpeg stdout is None for {self.name}")
                    self.available = False
                    break
                    
                raw = self.proc.stdout.read(frame_size)
                if len(raw) != frame_size:
                    consecutive_failures += 1
                    if consecutive_failures >= max_consecutive_failures:
                        self.log_error(f"Too many consecutive frame read failures for {self.name}")
                        self.available = False
                        break
                    continue
                    
                # Reset failure counter on successful read
                consecutive_failures = 0
                
                frame = np.frombuffer(raw, dtype=np.uint8).reshape((self.height, self.width, 3))
                with self.lock:
                    self.frame = frame
                    self.frame_count += 1
                    self.last_frame_update = time.time()
                    
                # Trigger ROS publishing if enabled
                if self.ros_publish_enabled:
                    self.frame_ready_event.set()
                    
            except Exception as e:
                consecutive_failures += 1
                self.log_error(f"Error reading frame for {self.name}: {e}")
                if consecutive_failures >= max_consecutive_failures:
                    self.log_error(f"Too many consecutive errors for {self.name}, marking as unavailable")
                    self.available = False
                    break
                time.sleep(0.1)  # Small delay before retry

    def get_frame(self):
        """Get current frame as a copy"""
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def resize_frame(self, frame, max_width):
        """Resize frame to specified max width while maintaining aspect ratio"""
        h, w = frame.shape[:2]
        if w > max_width:
            scale = max_width / w
            new_w = max_width
            new_h = int(h * scale)
            return cv2.resize(frame, (new_w, new_h))
        return frame

    def generate_frames_for_streaming(self, fps=25, quality=75, max_width=800):
        """Generator function for streaming frames to multiple viewers with minimal latency."""
        with self.viewer_lock:
            self.viewer_count += 1
            
        last_frame_id = -1  # Track frame changes to avoid re-encoding same frame
        
        try:
            while True:
                if not self.available:
                    # Generate error image for unavailable stream
                    error_img = np.zeros((240, 320, 3), dtype=np.uint8)
                    error_img[:] = (50, 50, 50)  # Dark gray background
                    
                    # Add text
                    cv2.putText(error_img, f"{self.name} Stream", (20, 80), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(error_img, "Not Available", (20, 125), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(error_img, self.url, (20, 170), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    
                    _, buffer = cv2.imencode('.jpg', error_img)
                    frame_bytes = buffer.tobytes()
                    
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                    time.sleep(1)  # Update error message every second
                    continue
                
                resized_frame = None
                with self.lock:
                    # Check if we have a new frame
                    if self.frame is not None and self.frame_count != last_frame_id:
                        # Resize frame to specified max width for lower latency
                        resized_frame = self.resize_frame(self.frame, max_width)
                        last_frame_id = self.frame_count
                
                # Only encode and send if we have a new frame
                if resized_frame is not None:
                    try:
                        # Use specified quality for encoding
                        _, buffer = cv2.imencode('.jpg', resized_frame, [cv2.IMWRITE_JPEG_QUALITY, max(quality - 20, 30)])
                        frame_bytes = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                    except Exception as e:
                        self.log_error(f"Error encoding frame for {self.name}: {e}")
                else:
                    # No new frame, just wait a bit
                    time.sleep(0.01)  # 10ms sleep when no new frame
                    
        except GeneratorExit:
            pass
        finally:
            with self.viewer_lock:
                self.viewer_count -= 1
    
    def get_frame_info(self):
        """Get frame info for reconnection detection."""
        with self.lock:
            return {
                'frame_count': self.frame_count,
                'last_update': self.last_frame_update,
                'has_frame': self.frame is not None
            }

    def restart(self):
        """restart the RTSP stream"""
        self.log_info(f"Restarting stream {self.name}...")
        self.stop()
        time.sleep(1)  
        self.__init__(self.name, self.url, self.node)  # reinitialize the stream
    
    def change_url(self, new_url):
        """Switch to a new RTSP URL"""
        self.log_info(f"Changing URL for {self.name} to: {new_url}")
        self.url = new_url
        self.restart()
    
    def is_running(self):
        """Check if the stream is running and has valid frames"""
        if not (self.running and self.available):
            return False
        
        # Check if we're actually receiving frames
        current_time = time.time()
        time_since_last_frame = current_time - self.last_frame_update
        
        # If no frame has been received in the last 10 seconds, consider it not running
        if time_since_last_frame > 10:
            return False
            
        # Check if we have valid frame data
        with self.lock:
            has_valid_frame = self.frame is not None
            
        return has_valid_frame

    def enable_ros_publishing(self, publisher, cv_bridge):
        """Enable event-driven ROS image publishing"""
        self.ros_publisher = publisher
        self.cv_bridge = cv_bridge
        self.ros_publish_enabled = True
        
        # Start ROS publishing thread
        self.ros_publish_thread = threading.Thread(target=self._ros_publish_loop, daemon=True)
        self.ros_publish_thread.start()
        self.log_info(f"Event-driven ROS publishing enabled for {self.name}")

    def disable_ros_publishing(self):
        """Disable ROS image publishing"""
        self.ros_publish_enabled = False
        self.ros_publisher = None
        self.cv_bridge = None
        self.log_info(f"ROS publishing disabled for {self.name}")

    def _ros_publish_loop(self):
        """Event-driven ROS publishing loop"""
        while self.running and self.ros_publish_enabled:
            # Wait for new frame event
            if self.frame_ready_event.wait(timeout=0.1):
                self.frame_ready_event.clear()
                
                # Check if we have a new frame to publish
                if self.frame_count > self.last_published_frame_id:
                    self._publish_ros_frame()
                    self.last_published_frame_id = self.frame_count

    def _publish_ros_frame(self):
        """Publish current frame as ROS Image message"""
        if not self.ros_publish_enabled or not self.ros_publisher or not self.cv_bridge:
            return
            
        frame = self.get_frame()
        if frame is not None:
            try:
                # Convert OpenCV image to ROS Image message
                ros_image = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
                if self.node:
                    ros_image.header.stamp = self.node.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera"
                
                # Publish the image
                self.ros_publisher.publish(ros_image)
            except Exception as e:
                self.log_error(f"Error publishing ROS image: {e}")

    def stop(self):
        """stop the RTSP stream"""
        if not self.running:
            return
            
        self.running = False
        if self.available and self.proc:
            try:
                self.proc.kill()
                self.proc.wait(timeout=3)  # Wait up to 3 seconds for process to terminate
            except subprocess.TimeoutExpired:
                self.log_warn(f"FFmpeg process for {self.name} did not terminate cleanly")
            except Exception as e:
                self.log_error(f"Error stopping FFmpeg process for {self.name}: {e}")
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=3)  # Wait up to 3 seconds for thread to finish
        
        with self.lock:
            self.frame = None
        
        self.log_info(f"Stream {self.name} stopped")


class CameraNode(Node):
    """ROS2 node for generic IP camera RTSP streaming and control."""
    
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("Initializing Generic Camera Node...")
        
        # Declare and read camera parameters
        self.declare_parameter('camera_name', 'GenericCamera')
        self.camera_name = self.get_parameter('camera_name').value
        self.get_logger().info(f"camera_name from param = {self.camera_name}")
        
        self.declare_parameter('rtsp_url_main', 'rtsp://admin:123456@192.168.1.100/stream0')
        self.rtsp_url_main = self.get_parameter('rtsp_url_main').value
        self.get_logger().info(f"rtsp_url_main from param = {self.rtsp_url_main}")
        
        self.declare_parameter('camera_ip', '192.168.1.100')
        self.camera_ip = self.get_parameter('camera_ip').value
        self.get_logger().info(f"camera_ip from param = {self.camera_ip}")
        
        # Declare Flask server port parameter
        self.declare_parameter('server_port', 8010)
        self.server_port = self.get_parameter('server_port').value
        self.get_logger().info(f"server_port from param = {self.server_port}")
        
        # Declare performance tuning parameters
        self.declare_parameter('stream_fps', 25)
        self.stream_fps = self.get_parameter('stream_fps').value
        self.get_logger().info(f"stream_fps from param = {self.stream_fps}")
        
        self.declare_parameter('jpeg_quality', 75)
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.get_logger().info(f"jpeg_quality from param = {self.jpeg_quality}")
        
        self.declare_parameter('max_width', 800)
        self.max_width = self.get_parameter('max_width').value
        self.get_logger().info(f"max_width from param = {self.max_width}")
        
        # Declare ROS2 publishing control parameter
        self.declare_parameter('publish_ros_image', True)
        self.publish_ros_image = self.get_parameter('publish_ros_image').value
        self.get_logger().info(f"publish_ros_image from param = {self.publish_ros_image}")
        
        # Declare ROS2 topic name parameter
        self.declare_parameter('ros_topic_name', '/camera/image_raw')
        self.ros_topic_name = self.get_parameter('ros_topic_name').value
        self.get_logger().info(f"ros_topic_name from param = {self.ros_topic_name}")
        
        # Setup RTSP URL - use only main stream
        self.rtsp_url = self.rtsp_url_main
        
        # latest snapshot path
        self.latest_snapshot = None
        
        # Initialize RTSP stream
        self.stream = None
        self.get_logger().info(f"Starting camera stream: {self.rtsp_url}")
        self.stream = RTSPStream(self.camera_name, self.rtsp_url, self)
        
        # Create ROS2 Image Publisher for camera (only if enabled)
        if self.publish_ros_image:
            # Create QoS profile optimized for real-time video streaming
            # Use RELIABLE delivery so downstream nodes receive all frames when possible
            camera_publisher_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,      # Deliver frames reliably to subscribers
                durability=DurabilityPolicy.VOLATILE,       # Don't store messages
                history=HistoryPolicy.KEEP_LAST,           # Only keep latest frame
                depth=1                                     # Only keep the most recent frame
            )
            
            self.camera_image_publisher = self.create_publisher(
                Image,
                self.ros_topic_name,
                camera_publisher_qos  # Use optimized QoS with explicit RELIABLE delivery
            )
            
            # CV Bridge for converting between OpenCV and ROS image formats
            self.bridge = CvBridge()
            
            # Enable event-driven ROS publishing instead of timer-based
            self.stream.enable_ros_publishing(self.camera_image_publisher, self.bridge)
            
            self.get_logger().info("ROS2 event-driven image publishing enabled")
            self.get_logger().info(f"ROS2 image topic: {self.ros_topic_name}")
        else:
            self.camera_image_publisher = None
            self.bridge = None
            self.get_logger().info("ROS2 image publishing disabled - only web interface available")
        
        # Create service for taking snapshots
        self.snapshot_service = self.create_service(
            Trigger,
            f'/{self.camera_name.lower()}/take_snapshot',
            self.take_snapshot_callback
        )
        
        # Create restart service
        self.restart_service = self.create_service(
            Trigger,
            f'/restart_{self.camera_name.lower()}_node',
            self.restart_callback
        )
        
        # Initialize Flask app for web interface
        self.flask_app = Flask(__name__)
        self.setup_flask_routes()
        
        # Start Flask server in a separate thread
        self.flask_thread = threading.Thread(
            target=self.run_flask_server,
            daemon=True
        )
        self.flask_thread.start()
        
        self.get_logger().info("Generic Camera Node initialized successfully")
        self.get_logger().info(f"Web interface available at http://localhost:{self.server_port}")
        if self.publish_ros_image:
            self.get_logger().info(f"ROS2 image topic: {self.ros_topic_name}")
        self.get_logger().info(f"Snapshot service: /{self.camera_name.lower()}/take_snapshot")
        self.get_logger().info(f"Restart service: /restart_{self.camera_name.lower()}_node")

    def take_snapshot_callback(self, request, response):
        """ROS2 service callback for taking snapshots"""
        try:
            if not self.stream or not self.stream.is_running():
                response.success = False
                response.message = "Camera stream is not available"
                return response
            
            frame = self.stream.get_frame()
            if frame is None:
                response.success = False
                response.message = "No frame available from camera"
                return response
            
            # Create snapshots directory if it doesn't exist
            snapshots_dir = f"/tmp/{self.camera_name.lower()}_snapshots"
            os.makedirs(snapshots_dir, exist_ok=True)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"snapshot_{timestamp}.jpg"
            filepath = os.path.join(snapshots_dir, filename)
            
            # Save the image
            success = cv2.imwrite(filepath, frame)
            
            if success:
                self.latest_snapshot = filepath
                response.success = True
                response.message = f"Snapshot saved: {filename}"
                self.get_logger().info(f"Snapshot saved: {filepath}")
            else:
                response.success = False
                response.message = "Failed to save snapshot"
                self.get_logger().error("Failed to save snapshot")
                
        except Exception as e:
            response.success = False
            response.message = f"Error taking snapshot: {str(e)}"
            self.get_logger().error(f"Error taking snapshot: {e}")
        
        return response

    def restart_callback(self, request, response):
        """Handle restart service request."""
        self.get_logger().info("Manual restart request received")
        
        # Set response
        response.success = True
        response.message = "Restarting camera node..."
        
        # Schedule restart in a separate thread to allow response to be sent
        def delayed_restart():
            import time
            time.sleep(1)  # Give time for response to be sent
            self.get_logger().info("Executing camera node restart...")
            try:
                # Restart the camera stream
                if self.stream:
                    self.stream.restart()
                    self.get_logger().info("Camera stream restarted successfully")
                else:
                    self.get_logger().warn("No camera stream to restart")
            except Exception as e:
                self.get_logger().error(f"Error during restart: {e}")
        
        import threading
        restart_thread = threading.Thread(target=delayed_restart)
        restart_thread.start()
        
        return response

    def setup_flask_routes(self):
        """Setup Flask routes for web interface"""
        
        @self.flask_app.route('/')
        def index():
            return f'''
            <html>
                <head>
                    <title>{self.camera_name} Control</title>
                    <style>
                        body {{
                            font-family: Arial, sans-serif;
                            max-width: 1000px;
                            margin: 0 auto;
                            padding: 20px;
                            background-color: #f5f5f5;
                        }}
                        .header {{
                            text-align: center;
                            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                            color: white;
                            padding: 20px;
                            border-radius: 10px;
                            margin-bottom: 20px;
                        }}
                        .controls {{
                            background: white;
                            padding: 20px;
                            border-radius: 10px;
                            margin-bottom: 20px;
                            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
                        }}
                        button {{
                            padding: 10px 20px;
                            margin: 5px;
                            font-size: 16px;
                            cursor: pointer;
                            border: none;
                            border-radius: 5px;
                            transition: background-color 0.3s;
                        }}
                        .start-btn {{ background-color: #4CAF50; color: white; }}
                        .start-btn:hover {{ background-color: #45a049; }}
                        .stop-btn {{ background-color: #f44336; color: white; }}
                        .stop-btn:hover {{ background-color: #da190b; }}
                        .restart-btn {{ background-color: #ff9800; color: white; }}
                        .restart-btn:hover {{ background-color: #e68900; }}
                        .snapshot-btn {{ background-color: #2196F3; color: white; }}
                        .snapshot-btn:hover {{ background-color: #0b7dda; }}
                        select {{
                            padding: 8px 12px;
                            margin: 5px;
                            font-size: 14px;
                            border: 1px solid #ddd;
                            border-radius: 4px;
                        }}
                        label {{
                            font-weight: bold;
                            margin-right: 10px;
                        }}
                        .status {{ 
                            padding: 15px; 
                            margin: 10px 0;
                            border-radius: 5px;
                            font-weight: bold;
                        }}
                        .status.running {{ background-color: #d4edda; color: #155724; border: 1px solid #c3e6cb; }}
                        .status.stopped {{ background-color: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }}
                        #camera-container {{
                            text-align: center;
                            background: white;
                            border: 2px solid #ddd;
                            padding: 10px;
                            border-radius: 10px;
                            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
                        }}
                        #camera-feed {{
                            max-width: 100%; 
                            height: auto;
                            border-radius: 5px;
                        }}
                        .info-panel {{
                            background: white;
                            padding: 15px;
                            border-radius: 10px;
                            margin-top: 20px;
                            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
                        }}
                        .info-panel h3 {{
                            margin-top: 0;
                            color: #333;
                        }}
                    </style>
                </head>
                <body>
                    <div class="header">
                        <h1>📹 {self.camera_name} Control</h1>
                        <p>Real-time RTSP Camera Streaming and Control Interface</p>
                    </div>
                    
                    <div class="controls">
                        <h3>Camera Controls</h3>
                        <button class="start-btn" onclick="startCamera()">▶️ Start Camera</button>
                        <button class="stop-btn" onclick="stopCamera()">⏹️ Stop Camera</button>
                        <button class="restart-btn" onclick="restartCamera()">🔄 Restart Camera</button>
                        <button class="snapshot-btn" onclick="takeSnapshot()">📸 Take Snapshot</button>
                        <button id="rosPublishBtn" class="snapshot-btn" onclick="toggleRosImagePublish()">🔄 Toggle ROS2 Publish</button>
                        
                        <br><br>
                        <div id="ros-publish-status" style="background: #f0f9ff; border: 1px solid #0ea5e9; border-radius: 6px; padding: 8px; margin-top: 10px;">
                            <strong>🤖 ROS2 Image Publishing:</strong> <span id="ros-publish-enabled">Checking...</span>
                        </div>
                    </div>

                    <div id="status" class="status">Checking status...</div>

                    <div id="camera-container">
                        <img id="camera-feed" src="/video_feed" alt="Camera Feed">
                    </div>

                    <div class="info-panel">
                        <h3>📋 System Information</h3>
                        <p><strong>Node Name:</strong> camera_node</p>
                        <p><strong>Camera Name:</strong> {self.camera_name}</p>
                        <p><strong>ROS2 Topic:</strong> {self.ros_topic_name}</p>
                        <p><strong>Snapshot Service:</strong> /{self.camera_name.lower()}/take_snapshot</p>
                        <p><strong>Restart Service:</strong> /restart_{self.camera_name.lower()}_node</p>
                        <p><strong>Server Port:</strong> {self.server_port}</p>
                        <p><strong>Camera IP:</strong> {self.camera_ip}</p>
                    </div>

                    <script>
                        function startCamera() {{
                            fetch('/start', {{method: 'POST'}})
                                .then(response => response.json())
                                .then(data => {{
                                    updateStatus(data.message, data.status);
                                    if (data.status === 'success') {{
                                        setTimeout(() => {{
                                            refreshImage();
                                        }}, 1000);
                                    }}
                                }});
                        }}
                        
                        function stopCamera() {{
                            fetch('/stop', {{method: 'POST'}})
                                .then(response => response.json())
                                .then(data => {{
                                    updateStatus(data.message, data.status);
                                }});
                        }}
                        
                        function restartCamera() {{
                            updateStatus('Restarting camera...', 'running');
                            
                            fetch('/restart', {{method: 'POST'}})
                                .then(response => response.json())
                                .then(data => {{
                                    updateStatus(data.message, data.status);
                                    if (data.status === 'success') {{
                                        setTimeout(() => {{
                                            refreshImage();
                                        }}, 2000); // Wait a bit longer for restart
                                    }}
                                }})
                                .catch(error => {{
                                    updateStatus('Error restarting camera: ' + error.message, 'error');
                                }});
                        }}
                        
                        function takeSnapshot() {{
                            updateStatus('Taking snapshot...', 'running');

                            fetch('/snapshot', {{method: 'POST'}})
                                .then(response => response.json())
                                .then(data => {{
                                    updateStatus(data.message, data.status);
                                    if (data.status === 'success') {{
                                        const a = document.createElement('a');
                                        a.href = '/download_snapshot';
                                        a.download = data.filename;
                                        a.click();
                                    }}
                                }});
                        }}
                        
                        function refreshImage() {{
                            const img = document.getElementById('camera-feed');
                            img.src = '/video_feed?' + new Date().getTime();
                        }}
                        
                        function getStatus() {{
                            fetch('/status')
                                .then(response => response.json())
                                .then(data => {{
                                    updateStatus(data.message, data.is_running ? 'running' : 'stopped');
                                }});
                        }}
                        
                        function updateStatus(message, status) {{
                            const statusDiv = document.getElementById('status');
                            statusDiv.textContent = message;
                            statusDiv.className = 'status ' + (status === 'success' || status === 'running' ? 'running' : 'stopped');
                        }}
                        
                        function toggleRosImagePublish() {{
                            updateStatus('Toggling ROS2 image publishing...', 'running');
                            
                            fetch('/ros_image_publish', {{method: 'POST', headers: {{'Content-Type': 'application/json'}}, body: JSON.stringify({{}})}})
                                .then(response => response.json())
                                .then(data => {{
                                    updateStatus(data.message, data.status);
                                    updateRosPublishStatus();
                                }})
                                .catch(error => {{
                                    updateStatus('Error toggling ROS2 publish: ' + error.message, 'error');
                                }});
                        }}
                        
                        function updateRosPublishStatus() {{
                            fetch('/ros_image_publish')
                                .then(response => response.json())
                                .then(data => {{
                                    const statusElement = document.getElementById('ros-publish-enabled');
                                    const btnElement = document.getElementById('rosPublishBtn');
                                    
                                    if (data.enabled) {{
                                        statusElement.textContent = 'Enabled ✅';
                                        statusElement.style.color = '#10b981';
                                        btnElement.textContent = '🔄 Disable ROS2 Publish';
                                        btnElement.className = 'stop-btn';
                                    }} else {{
                                        statusElement.textContent = 'Disabled ❌';
                                        statusElement.style.color = '#ef4444';
                                        btnElement.textContent = '🔄 Enable ROS2 Publish';
                                        btnElement.className = 'start-btn';
                                    }}
                                }})
                                .catch(error => {{
                                    document.getElementById('ros-publish-enabled').textContent = 'Error checking status';
                                }});
                        }}
                        
                        window.onload = function() {{
                            getStatus();
                            updateRosPublishStatus();
                        }};
                        
                        setInterval(getStatus, 5000);
                        setInterval(updateRosPublishStatus, 10000); // Check ROS2 publish status every 10 seconds
                    </script>
                </body>
            </html>
            '''

        @self.flask_app.route('/video_feed')
        def video_feed():
            if self.stream and self.stream.is_running():
                return Response(
                    self.stream.generate_frames_for_streaming(
                        fps=self.stream_fps,
                        quality=self.jpeg_quality,
                        max_width=self.max_width
                    ),
                    mimetype='multipart/x-mixed-replace; boundary=frame'
                )
            else:
                # Return a simple error image
                error_img = np.zeros((240, 320, 3), dtype=np.uint8)
                error_img[:] = (50, 50, 50)
                cv2.putText(error_img, "Camera Not Available", (20, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                _, buffer = cv2.imencode('.jpg', error_img)
                
                def generate_error():
                    while True:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                        time.sleep(1)
                
                return Response(generate_error(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.flask_app.route('/start', methods=['POST'])
        def start_camera():
            try:
                if self.stream and self.stream.is_running():
                    return jsonify({'status': 'success', 'message': 'Camera is already running'})
                
                # Restart stream if not running
                if self.stream:
                    self.stream.restart()
                else:
                    self.stream = RTSPStream(self.camera_name, self.rtsp_url, self)
                
                if self.stream.is_running():
                    return jsonify({'status': 'success', 'message': 'Camera started successfully'})
                else:
                    return jsonify({'status': 'error', 'message': 'Failed to start camera'})
                    
            except Exception as e:
                self.get_logger().error(f"Error starting camera: {e}")
                return jsonify({'status': 'error', 'message': f'Error starting camera: {str(e)}'})

        @self.flask_app.route('/stop', methods=['POST'])
        def stop_camera():
            try:
                if self.stream:
                    self.stream.stop()
                    return jsonify({'status': 'success', 'message': 'Camera stopped successfully'})
                else:
                    return jsonify({'status': 'success', 'message': 'Camera was not running'})
                    
            except Exception as e:
                self.get_logger().error(f"Error stopping camera: {e}")
                return jsonify({'status': 'error', 'message': f'Error stopping camera: {str(e)}'})

        @self.flask_app.route('/restart', methods=['POST'])
        def restart_camera():
            try:
                self.get_logger().info("Manual restart request received from web interface")
                
                if self.stream:
                    self.stream.restart()
                    
                    # Wait a moment and check if restart was successful
                    import time
                    time.sleep(1)
                    
                    if self.stream.is_running():
                        return jsonify({'status': 'success', 'message': 'Camera restarted successfully'})
                    else:
                        return jsonify({'status': 'error', 'message': 'Failed to restart camera stream'})
                else:
                    # If no stream exists, create a new one
                    self.stream = RTSPStream(self.camera_name, self.rtsp_url, self)
                    
                    if self.stream.is_running():
                        return jsonify({'status': 'success', 'message': 'Camera started successfully'})
                    else:
                        return jsonify({'status': 'error', 'message': 'Failed to start camera'})
                    
            except Exception as e:
                self.get_logger().error(f"Error restarting camera: {e}")
                return jsonify({'status': 'error', 'message': f'Error restarting camera: {str(e)}'})

        @self.flask_app.route('/status')
        def get_camera_status():
            if not self.stream:
                return jsonify({'is_running': False, 'message': 'Camera stream not initialized'})
            
            is_running = self.stream.is_running()
            
            # Get detailed status information for error cases
            if not is_running:
                current_time = time.time()
                time_since_last_frame = current_time - self.stream.last_frame_update
                
                with self.stream.lock:
                    has_frame = self.stream.frame is not None
            
            if is_running:
                message = 'Camera is running'
            else:
                if not self.stream.available:
                    message = 'Camera connection failed - unable to start FFmpeg process'
                elif not has_frame:
                    message = 'Camera process running but no frames received - check camera connection'
                elif time_since_last_frame > 10:
                    message = f'Camera connection lost - no frames for {time_since_last_frame:.1f} seconds'
                else:
                    message = 'Camera is stopped'
                    
            return jsonify({'is_running': is_running, 'message': message})

        @self.flask_app.route('/snapshot', methods=['POST'])
        def take_snapshot():
            try:
                if not self.stream or not self.stream.is_running():
                    return jsonify({'status': 'error', 'message': 'Camera is not running'})
                
                frame = self.stream.get_frame()
                if frame is None:
                    return jsonify({'status': 'error', 'message': 'No frame available'})
                
                # Create snapshots directory if it doesn't exist
                snapshots_dir = f"/tmp/{self.camera_name.lower()}_snapshots"
                os.makedirs(snapshots_dir, exist_ok=True)
                
                # Generate filename with timestamp
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"snapshot_{timestamp}.jpg"
                filepath = os.path.join(snapshots_dir, filename)
                
                # Save the image
                success = cv2.imwrite(filepath, frame)
                
                if success:
                    self.latest_snapshot = filepath
                    return jsonify({
                        'status': 'success',
                        'message': 'Snapshot saved successfully',
                        'filename': filename
                    })
                else:
                    return jsonify({'status': 'error', 'message': 'Failed to save snapshot'})
                    
            except Exception as e:
                self.get_logger().error(f"Error taking snapshot: {e}")
                return jsonify({'status': 'error', 'message': f'Error taking snapshot: {str(e)}'})

        @self.flask_app.route('/download_snapshot')
        def download_snapshot():
            try:
                if self.latest_snapshot and os.path.exists(self.latest_snapshot):
                    from flask import send_file
                    return send_file(self.latest_snapshot, as_attachment=True)
                else:
                    return jsonify({'status': 'error', 'message': 'No snapshot available'})
                    
            except Exception as e:
                self.get_logger().error(f"Error downloading snapshot: {e}")
                return jsonify({'status': 'error', 'message': f'Error downloading snapshot: {str(e)}'})

        @self.flask_app.route('/ros_image_publish', methods=['GET'])
        def get_ros_image_publish_status():
            """Get current ROS2 image publish status"""
            return jsonify({
                'status': 'success',
                'enabled': self.publish_ros_image,
                'message': 'ROS2 image publishing is ' + ('enabled' if self.publish_ros_image else 'disabled')
            })

        @self.flask_app.route('/ros_image_publish', methods=['POST'])
        def toggle_ros_image_publish():
            """Toggle ROS2 image publishing on/off"""
            try:
                data = request.get_json()
                enable = data.get('enable', not self.publish_ros_image)
                
                if enable and not self.publish_ros_image:
                    # Enable ROS2 publishing
                    self.publish_ros_image = True
                    
                    # Create image publisher if not exists
                    if not self.camera_image_publisher:
                        # Create QoS profile optimized for real-time video streaming
                        camera_publisher_qos = QoSProfile(
                            reliability=ReliabilityPolicy.RELIABLE,      # Deliver frames reliably
                            durability=DurabilityPolicy.VOLATILE,       # Don't store messages
                            history=HistoryPolicy.KEEP_LAST,           # Only keep latest frame
                            depth=1                                     # Only keep the most recent frame
                        )
                        
                        self.camera_image_publisher = self.create_publisher(
                            Image,
                            self.ros_topic_name,
                            camera_publisher_qos  # Use optimized QoS
                        )
                    
                    # Create CV bridge if not exists
                    if not self.bridge:
                        self.bridge = CvBridge()
                    
                    # Enable event-driven ROS publishing on stream
                    if self.stream:
                        self.stream.enable_ros_publishing(self.camera_image_publisher, self.bridge)
                    
                    self.get_logger().info("ROS2 event-driven image publishing enabled")
                    return jsonify({
                        'status': 'success',
                        'enabled': True,
                        'message': 'ROS2 image publishing enabled successfully'
                    })
                    
                elif not enable and self.publish_ros_image:
                    # Disable ROS2 publishing
                    self.publish_ros_image = False
                    
                    # Disable event-driven publishing on stream
                    if self.stream:
                        self.stream.disable_ros_publishing()
                    
                    self.get_logger().info("ROS2 image publishing disabled")
                    return jsonify({
                        'status': 'success',
                        'enabled': False,
                        'message': 'ROS2 image publishing disabled successfully'
                    })
                
                else:
                    # No change needed
                    return jsonify({
                        'status': 'success',
                        'enabled': self.publish_ros_image,
                        'message': f'ROS2 image publishing already {"enabled" if self.publish_ros_image else "disabled"}'
                    })
                    
            except Exception as e:
                self.get_logger().error(f"Error toggling ROS2 image publish: {e}")
                return jsonify({'status': 'error', 'message': f'Error toggling ROS2 image publish: {str(e)}'})

    def run_flask_server(self):
        """Run Flask server in a separate thread"""
        self.get_logger().info(f"Starting Flask server on port {self.server_port}...")
        try:
            self.flask_app.run(host='0.0.0.0', port=self.server_port, threaded=True, debug=False)
        except Exception as e:
            self.get_logger().error(f"Error running Flask server: {e}")

    def destroy_node(self):
        """Clean up resources when shutting down"""
        self.get_logger().info("Shutting down Generic Camera Node...")
        
        # Stop the camera stream
        if self.stream:
            self.stream.stop()
        
        # Call parent destroy_node
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down camera node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

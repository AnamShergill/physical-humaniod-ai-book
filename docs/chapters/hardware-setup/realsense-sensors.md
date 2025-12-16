---
sidebar_label: 'Realsense Sensors Setup'
---

# Realsense Sensors Setup for Robotics Applications

## Overview

Intel RealSense sensors provide high-quality depth sensing, stereo vision, and motion tracking capabilities essential for robotics applications. This guide covers the selection, installation, configuration, and integration of RealSense sensors with robotics platforms including RTX workstations and Jetson edge devices.

## Learning Objectives

By the end of this lesson, you will be able to:
- Select appropriate RealSense sensors for specific robotics applications
- Install and configure RealSense sensors with proper drivers
- Integrate RealSense sensors with ROS/ROS2
- Calibrate and optimize sensor performance
- Process RealSense data for robotics applications

## RealSense Sensor Selection

### Product Line Overview

#### D400 Series (Stereo Depth)
```yaml
D415:
  Type: Stereo Depth
  Resolution: 1280×720 (depth), 1920×1080 (color)
  FPS: Up to 90
  Range: 0.3m - 10m
  FOV: 85.2° (H) × 58° (V) × 101.3° (D)
  Use Case: General purpose depth sensing

D435:
  Type: Stereo Depth
  Resolution: 1280×720 (depth), 1920×1080 (color)
  FPS: Up to 90
  Range: 0.2m - 10m
  FOV: 86° (H) × 56° (V) × 94° (D)
  Use Case: Robotics, AR/VR applications

D435i:
  Type: Stereo Depth + IMU
  Resolution: 1280×720 (depth), 1920×1080 (color)
  FPS: Up to 90
  Range: 0.2m - 10m
  IMU: Accelerometer + Gyroscope
  Use Case: SLAM, motion tracking

D455:
  Type: Stereo Depth (Long Range)
  Resolution: 2880×1620 (depth), 1920×1080 (color)
  FPS: Up to 30
  Range: 0.1m - 20m
  Accuracy: ±1% at 10m
  Use Case: Long-range applications
```

#### L500 Series (LiDAR)
```yaml
L515:
  Type: LiDAR Depth
  Resolution: 1024×768
  FPS: Up to 90
  Range: 0.25m - 9m
  Accuracy: ±1% at 1m, ±3% at 4m
  Power: USB-C (low power)
  Use Case: Indoor applications, robotics
```

### Selection Guidelines

```python
# Decision matrix for RealSense sensor selection

def select_realsense_sensor(application_requirements):
    """
    Select appropriate RealSense sensor based on application requirements
    """
    requirements = {
        'distance_range': 'short/medium/long',
        'indoor_outdoor': 'indoor/outdoor',
        'accuracy_needed': 'low/medium/high',
        'power_constraints': 'low/medium/high',
        'processing_power': 'limited/adequate',
        'imu_required': True/False
    }

    if requirements['distance_range'] == 'long' and requirements['indoor_outdoor'] == 'indoor':
        return 'D455'  # Long range stereo
    elif requirements['distance_range'] == 'medium' and requirements['imu_required']:
        return 'D435i'  # Stereo with IMU
    elif requirements['power_constraints'] == 'low' and requirements['distance_range'] == 'short':
        return 'L515'  # LiDAR (low power)
    else:
        return 'D435'  # General purpose

# Example usage
robotics_app = {
    'distance_range': 'medium',
    'indoor_outdoor': 'indoor',
    'accuracy_needed': 'high',
    'power_constraints': 'medium',
    'processing_power': 'adequate',
    'imu_required': True
}

recommended_sensor = select_realsense_sensor(robotics_app)
print(f"Recommended sensor: {recommended_sensor}")
```

## Driver Installation and Setup

### Ubuntu Installation

```bash
# Add RealSense repository
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# Install RealSense SDK
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils librealsense2-dev librealsense2-dbg

# Verify installation
rs-enumerate-devices
```

### From Source Installation (Latest Features)

```bash
# Install dependencies
sudo apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# Clone and build from source
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout tags/v2.54.2 -b v2.54.2  # Use latest stable tag

# Build with patches for better compatibility
sudo ./scripts/setup_udev_rules.sh
sudo ./scripts/rsudev.sh

mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
make -j4
sudo make install
```

### Jetson Platform Installation

```bash
# For Jetson platforms, use pre-compiled packages
# Download from Intel RealSense GitHub releases for Jetson

# Install dependencies
sudo apt install -y libssl-dev libusb-1.0-0-dev libgtk-3-dev libglfw3-dev

# Download and install Jetson-specific package
wget https://github.com/IntelRealSense/librealsense/releases/download/v2.54.2/librealsense2_2.54.2-0~realsense0.5378_arm64.deb
sudo dpkg -i librealsense2_2.54.2-0~realsense0.5378_arm64.deb
```

## Hardware Configuration

### USB Power and Bandwidth Requirements

```bash
# Check USB configuration
lsusb -t

# For D400 series sensors
# USB 3.2 Gen 1 (5Gbps) minimum recommended
# USB 3.2 Gen 2 (10Gbps) for multiple sensors
# Power: 4.5W typical, up to 6W maximum

# Check USB bandwidth usage
cat /sys/kernel/debug/usb/devices | grep -A 10 -B 1 RealSense
```

### Multiple Sensor Configuration

```bash
# For multiple sensors, ensure adequate USB bandwidth
# Use separate USB controllers when possible

# Check connected sensors
rs-enumerate-devices -c  # Get camera info

# Example for 2 sensors configuration
cat > multi_sensor_config.json << 'EOF'
{
  "devices": [
    {
      "serial": "123456789012",
      "name": "sensor_left",
      "resolution": [1280, 720],
      "fps": 30
    },
    {
      "serial": "098765432109",
      "name": "sensor_right",
      "resolution": [1280, 720],
      "fps": 30
    }
  ]
}
EOF
```

## ROS/ROS2 Integration

### RealSense ROS Package Installation

```bash
# Install RealSense ROS2 package
cd ~/robot_ws/src
git clone -b humble https://github.com/IntelRealSense/realsense-ros.git
cd ~/robot_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select realsense2_camera realsense2_description
source install/setup.bash
```

### Basic Launch Configuration

```xml
<!-- realsense.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    device_type_arg = DeclareLaunchArgument(
        'device_type', default_value='d435',
        description='Device type: d415, d435, d435i, d455, l515'
    )

    return LaunchDescription([
        device_type_arg,

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[{
                'device_type': [LaunchConfiguration('device_type')],
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'enable_pointcloud': True,
                'pointcloud_texture_stream': 'RS2_STREAM_COLOR',
                'pointcloud_texture_index': 0,
                'enable_sync': True,
                'align_depth.enable': True,
            }],
            output='screen'
        )
    ])
```

### Advanced Configuration Parameters

```yaml
# realsense_config.yaml
camera:
  ros__parameters:
    # Device settings
    device_type: "d435i"
    serial_no: ""
    usb_port_id: ""
    device_type: "d435"

    # Enable streams
    enable_color: true
    enable_depth: true
    enable_infra1: false
    enable_infra2: false
    enable_fisheye1: false
    enable_fisheye2: false
    enable_gyro: true
    enable_accel: true

    # Image settings
    depth_module.profile: "640x480x30"
    rgb_camera.profile: "640x480x30"
    fisheye_width: 848
    fisheye_height: 480
    fisheye_fps: 30

    # Processing settings
    enable_pointcloud: true
    enable_sync: true
    align_depth.enable: true

    # Depth settings
    depth_module.depth_clamp_max: 10.0
    depth_module.depth_units: 0.001
    depth_module.enable_auto_exposure: true
    depth_module.exposure: 8500
    depth_module.gain: 16

    # RGB settings
    rgb_camera.enable_auto_exposure: true
    rgb_camera.exposure: 156
    rgb_camera.gain: 16
    rgb_camera.white_balance: 4600
```

## Calibration and Optimization

### Intrinsic Calibration

```bash
# Use RealSense tools for calibration
# For custom calibration:
rs-config -d <device_serial> --preset HighAccuracy

# Or programmatically:
cat > calibrate_camera.py << 'EOF'
import pyrealsense2 as rs
import numpy as np

def calibrate_realsense_intrinsics():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Start streaming
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)

    # Get stream profiles
    profile = pipeline.get_active_profile()
    depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()

    # Get intrinsic parameters
    depth_intrinsics = depth_profile.get_intrinsics()
    color_intrinsics = color_profile.get_intrinsics()

    print(f"Depth intrinsics: {depth_intrinsics}")
    print(f"Color intrinsics: {color_intrinsics}")

    pipeline.stop()

    return depth_intrinsics, color_intrinsics

# Run calibration
depth_int, color_int = calibrate_realsense_intrinsics()
EOF
```

### Stereo Baseline and Extrinsics

```python
# Calculate stereo baseline and extrinsics
import pyrealsense2 as rs
import numpy as np

def get_stereo_extrinsics():
    # Get extrinsic parameters between depth and color
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)

    # Get extrinsics
    depth_to_color_extrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(
        profile.get_stream(rs.stream.color).as_video_stream_profile()
    )

    print(f"Depth to Color Extrinsics: {depth_to_color_extrinsics}")

    pipeline.stop()
    return depth_to_color_extrinsics

# Get extrinsics
extrinsics = get_stereo_extrinsics()
```

## Data Processing for Robotics

### Depth Data Processing

```python
import pyrealsense2 as rs
import numpy as np
import cv2

class RealSenseProcessor:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configure streams
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start pipeline
        self.pipeline.start(self.config)

        # Create align object
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

    def get_aligned_frames(self):
        """Get aligned depth and color frames"""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None

        # Convert to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return depth_image, color_image

    def process_depth_for_navigation(self, depth_image, min_distance=0.3, max_distance=5.0):
        """Process depth data for navigation applications"""
        # Convert depth to meters
        depth_meters = depth_image * 0.001  # Assuming millimeter units

        # Create occupancy grid
        occupancy_grid = np.zeros_like(depth_meters)

        # Mark obstacles
        occupancy_grid[(depth_meters > min_distance) & (depth_meters < max_distance)] = 100
        occupancy_grid[depth_meters <= min_distance] = -1  # Unknown/invalid
        occupancy_grid[depth_meters >= max_distance] = 0   # Free space

        return occupancy_grid

    def extract_pointcloud(self, depth_frame, color_frame):
        """Extract point cloud from depth and color frames"""
        # Create point cloud object
        pc = rs.pointcloud()
        points = pc.calculate(depth_frame)

        # Map color texture to point cloud
        pc.map_to(color_frame)

        # Convert to numpy array
        points = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

        return points

# Example usage
processor = RealSenseProcessor()

try:
    while True:
        depth_img, color_img = processor.get_aligned_frames()
        if depth_img is not None and color_img is not None:
            # Process for navigation
            occupancy = processor.process_depth_for_navigation(depth_img)

            # Display images
            cv2.imshow('Depth', depth_img / np.max(depth_img))  # Normalize for display
            cv2.imshow('Color', color_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
finally:
    processor.pipeline.stop()
    cv2.destroyAllWindows()
```

### Integration with Navigation Stack

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealSenseNavigationNode(Node):
    def __init__(self):
        super().__init__('realsense_navigation')

        # Initialize RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.pipeline.start(self.config)

        # ROS publishers
        self.bridge = CvBridge()
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/camera/depth/points', 10)

        # Timer for data publishing
        self.timer = self.create_timer(0.1, self.publish_sensor_data)  # 10 Hz

    def publish_sensor_data(self):
        """Publish RealSense data to ROS topics"""
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if depth_frame:
            # Convert to ROS Image message
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = "camera_depth_frame"

            self.depth_pub.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization

### Multi-threaded Processing

```python
import threading
import queue
import pyrealsense2 as rs
import numpy as np

class ThreadedRealSenseProcessor:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.frame_queue = queue.Queue(maxsize=2)
        self.running = True

        # Start pipeline
        self.pipeline.start(self.config)

        # Start processing thread
        self.process_thread = threading.Thread(target=self._process_frames)
        self.process_thread.start()

    def _process_frames(self):
        """Process frames in separate thread"""
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                if not self.frame_queue.full():
                    self.frame_queue.put(frames)
            except:
                continue

    def get_processed_data(self):
        """Get processed data from queue"""
        try:
            frames = self.frame_queue.get_nowait()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if depth_frame and color_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                return depth_image, color_image
        except queue.Empty:
            return None, None

    def stop(self):
        """Stop processing"""
        self.running = False
        self.process_thread.join()
        self.pipeline.stop()

# Example usage
processor = ThreadedRealSenseProcessor()

try:
    while True:
        depth, color = processor.get_processed_data()
        if depth is not None:
            # Process data
            print(f"Depth shape: {depth.shape}, Color shape: {color.shape}")
finally:
    processor.stop()
```

### Power and Thermal Management

```bash
# Monitor RealSense power consumption
# Check USB power usage
cat /sys/kernel/debug/usb/devices | grep -A 5 -B 5 RealSense

# Thermal monitoring script
cat > realsense_thermal_monitor.sh << 'EOF'
#!/bin/bash

while true; do
    # Check if RealSense is connected and active
    if lsusb | grep -q "8086:0b07\|8086:0b08\|8086:0b09"; then
        # Monitor system temperature
        temp=$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | head -1)
        temp_c=$((temp / 1000))

        echo "RealSense active, System temp: ${temp_c}°C"

        # If temperature is high, reduce frame rate
        if [ $temp_c -gt 70 ]; then
            echo "High temperature detected, consider reducing frame rate"
        fi
    fi

    sleep 5
done
EOF

chmod +x realsense_thermal_monitor.sh
```

## Troubleshooting Common Issues

### USB Bandwidth Issues

```bash
# Check USB bandwidth usage
sudo apt install -y usbutils

# List USB devices and bandwidth
lsusb -t
cat /sys/kernel/debug/usb/devices

# Reduce resolution/frame rate to lower bandwidth
# In launch file or configuration:
# depth_module.profile: "640x480x15"  # Lower FPS
# rgb_camera.profile: "640x480x15"    # Lower FPS
```

### Sensor Not Detected

```bash
# Check if sensor is properly connected
lsusb | grep -i intel

# Check device permissions
ls -la /dev/bus/usb/*/*

# Fix permissions if needed
sudo chmod 666 /dev/bus/usb/*/*

# Or add udev rules
sudo cp /etc/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Depth Quality Issues

```python
# Adjust depth quality settings programmatically
def improve_depth_quality(pipeline):
    """Improve depth quality by adjusting settings"""
    # Get depth sensor
    sensor = pipeline.get_active_profile().get_device().first_depth_sensor()

    # Enable high density preset
    sensor.set_option(rs.option.visual_preset, rs.visual_preset.high_accuracy)

    # Adjust filters
    sensor.set_option(rs.option.filter_magnitude, 5)  # Temporal filter
    sensor.set_option(rs.option.filter_smooth_alpha, 0.5)
    sensor.set_option(rs.option.filter_smooth_delta, 20)

    print("Depth quality settings adjusted")
```

## Practical Exercise

Set up and test a complete RealSense sensor integration:

1. Install RealSense drivers and ROS package
2. Connect and verify sensor detection
3. Launch sensor with ROS integration
4. Process depth data for navigation applications
5. Optimize settings for your specific use case
6. Test with navigation stack integration

## Summary

RealSense sensors provide high-quality depth and visual data essential for robotics applications. Proper installation, configuration, and optimization enable effective use in navigation, mapping, and perception tasks. The integration with ROS/ROS2 makes them particularly valuable for robotics development.

## Next Steps

Continue with Cloud Simulation setup, which provides remote access to powerful simulation platforms and enables collaborative development of robotics applications without requiring local high-end hardware.
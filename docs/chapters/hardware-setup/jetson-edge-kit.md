---
sidebar_label: 'Jetson Edge Kit Setup'
---

# Jetson Edge Kit Setup for Robotics Applications

## Overview

NVIDIA Jetson platforms provide powerful, energy-efficient computing solutions for edge robotics applications. This guide covers the setup, configuration, and optimization of Jetson edge kits for robotics development, focusing on real-time processing, AI inference, and sensor integration.

## Learning Objectives

By the end of this lesson, you will be able to:
- Select appropriate Jetson hardware for specific robotics applications
- Configure Jetson devices for robotics workloads
- Install and optimize ROS/ROS2 for edge computing
- Deploy AI models on Jetson platforms
- Optimize power consumption and thermal management

## Jetson Platform Selection

### Jetson Product Line Comparison

#### Jetson Nano
```yaml
Specifications:
  GPU: 128-core Maxwell
  CPU: Quad-core ARM A57
  Memory: 4GB LPDDR4
  Power: 5-10W
  Use Case: Basic AI inference, simple robotics

Limitations:
  - Limited VRAM (4GB shared)
  - Not suitable for complex simulations
  - Limited parallel processing
```

#### Jetson Xavier NX
```yaml
Specifications:
  GPU: 384-core Volta with Tensor Cores
  CPU: Hexa-core ARM Carmel
  Memory: 8GB LPDDR4x
  Power: 10-15W
  Use Case: Moderate AI workloads, multi-sensor fusion

Advantages:
  - Good balance of performance and power
  - Suitable for mobile robotics
  - Adequate for real-time inference
```

#### Jetson AGX Orin
```yaml
Specifications:
  GPU: 2048-core Ada Lovelace
  CPU: 12-core ARM v8.2
  Memory: 32GB LPDDR5
  Power: 15-60W
  Use Case: Complex AI models, high-performance robotics

Advantages:
  - High performance for complex algorithms
  - Large memory capacity
  - Multiple camera inputs supported
```

### Platform Selection Guidelines

```python
# Decision matrix for Jetson platform selection

def select_jetson_platform(robotics_application):
    requirements = {
        'ai_inference_complexity': 'simple/complex',
        'sensor_types': ['camera', 'lidar', 'imu'],
        'power_budget': 'low/medium/high',
        'form_factor': 'compact/portable',
        'real_time_requirements': 'yes/no'
    }

    if requirements['ai_inference_complexity'] == 'simple' and requirements['power_budget'] == 'low':
        return 'Jetson Nano'
    elif requirements['ai_inference_complexity'] == 'moderate' and requirements['power_budget'] == 'medium':
        return 'Jetson Xavier NX'
    elif requirements['ai_inference_complexity'] == 'complex' and requirements['power_budget'] == 'high':
        return 'Jetson AGX Orin'
    else:
        return 'Consider specific requirements'

# Example usage
application = {
    'ai_inference_complexity': 'moderate',
    'sensor_types': ['camera', 'imu'],
    'power_budget': 'medium',
    'form_factor': 'compact',
    'real_time_requirements': 'yes'
}

recommended_platform = select_jetson_platform(application)
print(f"Recommended platform: {recommended_platform}")
```

## Jetson System Setup

### Initial Setup and Configuration

```bash
# Flash Jetson with appropriate image
# Download from NVIDIA Developer website

# Using Jetson Flash Tool
sudo ./jetson-flash \
  --device jetson-agx-orin-devkit \
  --flash-to-internal \
  --no-check \
  --accept-eula

# Or using SDK Manager (GUI tool)
# Download NVIDIA SDK Manager
# Follow guided installation process
```

### Basic System Configuration

```bash
# After initial boot, configure system
sudo apt update && sudo apt upgrade -y

# Install essential packages
sudo apt install -y \
  build-essential \
  cmake \
  git \
  vim \
  htop \
  iotop \
  python3-dev \
  python3-pip \
  python3-venv

# Configure timezone and locale
sudo dpkg-reconfigure tzdata
sudo dpkg-reconfigure locales
```

### Jetson Performance Mode

```bash
# Set Jetson to maximum performance mode
sudo nvpmodel -m 0  # Maximum performance mode

# Check current mode
sudo nvpmodel -q

# Apply fan control (if applicable)
sudo jetson_clocks  # Lock clocks to maximum frequency
```

## ROS/ROS2 Installation on Jetson

### ROS2 Humble Hawksbill Setup

```bash
# Install ROS2 Humble on Jetson (Ubuntu 22.04)
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-humble-ros-base
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Install additional robotics packages
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-robot-localization ros-humble-interactive-markers
sudo apt install -y ros-humble-usb-cam ros-humble-pointcloud-to-laserscan
```

### ROS Environment Setup

```bash
# Set up ROS environment
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc  # Set ROS domain for multi-robot systems
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc  # Better performance

# Create workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws
colcon build --symlink-install

echo 'source ~/robot_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## AI Framework Configuration

### TensorRT Installation and Optimization

```bash
# Install TensorRT for optimized inference
sudo apt install -y libnvinfer8 libnvonnxparsers8 libnvparsers8
sudo apt install -y python3-libnvinfer

# Verify TensorRT installation
python3 -c "import tensorrt as trt; print(f'TensorRT version: {trt.__version__}')"
```

### Deep Learning Framework Setup

```bash
# Install PyTorch optimized for Jetson
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install TensorFlow (optimized version for Jetson)
pip3 install --extra-index-url https://pypi.ngc.nvidia.com tensorflow==2.13.0+nv23.05

# Install OpenCV with CUDA support
sudo apt install -y python3-opencv libopencv-dev

# Install other ML libraries
pip3 install numpy scipy scikit-learn pillow
```

### Jetson Inference Setup

```bash
# Install NVIDIA's Jetson Inference framework
git clone --recursive https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../
make -j$(nproc)

# Install Python bindings
cd aarch64/bin
sudo ./install-pytorch.sh
```

## Sensor Integration

### Camera Setup

```bash
# Install camera support packages
sudo apt install -y v4l-utils libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly

# Test camera connection
v4l2-ctl --list-devices
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! autovideosink

# For CSI cameras (Jetson built-in)
# Test with:
# gst-launch-1.0 nvarguscamerasrc ! nvvidconv ! video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 ! videoconvert ! autovideosink
```

### LiDAR Integration

```bash
# Install ROS packages for common LiDAR types
sudo apt install -y ros-humble-scan-tools ros-humble-laser-filters
sudo apt install -y ros-humble-rplidar-ros

# For specific LiDAR models:
# Hokuyo URG: sudo apt install -y ros-humble-urg-node
# Velodyne: sudo apt install -y ros-humble-velodyne
# Ouster: sudo apt install -y ros-humble-ouster-msgs

# Example launch file for RPLIDAR
cat > ~/robot_ws/src/rplidar.launch.py << 'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen'
        )
    ])
EOF
```

## Power and Thermal Management

### Power Consumption Optimization

```bash
# Monitor power consumption
sudo tegrastats  # Real-time power, temperature, utilization

# Power mode configuration
# Create power optimization script
cat > power_optimization.sh << 'EOF'
#!/bin/bash

# Set to low power mode when possible
sudo nvpmodel -m 1  # Low power mode

# Disable unused interfaces
echo '0' | sudo tee /sys/class/bluetooth/hci0/power/control
echo 'auto' | sudo tee /sys/bus/pci/devices/0000:00:00.0/power/control

# CPU frequency scaling
echo 'ondemand' | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# GPU frequency scaling
echo 'simple_ondemand' | sudo tee /sys/class/devfreq/gpufreq/governor
EOF

chmod +x power_optimization.sh
```

### Thermal Management

```bash
# Install thermal management tools
sudo apt install -y thermald

# Configure thermal settings
cat > /etc/thermald/thermal-conf.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<ThermalConfiguration>
  <Platform>
    <Name>Jetson Custom Platform</Name>
    <ProductName>*</ProductName>
    <Preference>QUIET</Preference>
    <ThermalZones>
      <ThermalZone>
        <Type>cpu</Type>
        <TripPoints>
          <TripPoint>
            <Temperature>70000</Temperature>
            <type>passive</type>
            <ControlType>Configurable</ControlType>
          </TripPoint>
        </TripPoints>
      </ThermalZone>
    </ThermalZones>
  </Platform>
</ThermalConfiguration>
EOF

# Start thermal daemon
sudo systemctl enable thermald
sudo systemctl start thermald
```

## Networking and Communication

### Network Configuration for Robotics

```bash
# Configure network for robotics applications
# Static IP for robot (example)
cat > /etc/netplan/01-robot-network.yaml << 'EOF'
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: false
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      mtu: 9000  # Jumbo frames for high-bandwidth sensor data
EOF

sudo netplan apply
```

### Real-time Communication Setup

```bash
# Install and configure real-time communication
sudo apt install -y ros-humble-rt-test ros-humble-diagnostic-updater

# Configure DDS for real-time performance
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastdds_profiles.xml' >> ~/.bashrc

# Create FastDDS configuration
cat > ~/fastdds_profiles.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<dds>
  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>CustomUdpTransport</transport_id>
        <type>UDPv4</type>
        <sendBufferSize>65536</sendBufferSize>
        <receiveBufferSize>65536</receiveBufferSize>
      </transport_descriptor>
    </transport_descriptors>
  </profiles>
</dds>
EOF
```

## Performance Optimization

### System-Level Optimizations

```bash
# Optimize system for robotics workloads
# Reduce swappiness for better real-time performance
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p

# Optimize I/O scheduler
echo 'none' | sudo tee /sys/block/mmcblk0/queue/scheduler
echo 'none' | sudo tee /sys/block/mmcblk1/queue/scheduler

# Disable unnecessary services
sudo systemctl disable bluetooth
sudo systemctl disable avahi-daemon
sudo systemctl disable ModemManager
```

### Jetson-Specific Optimizations

```python
# Python script for Jetson performance optimization
import subprocess
import os

def optimize_jetson_performance():
    """Apply Jetson-specific optimizations for robotics"""

    # Set maximum performance mode
    subprocess.run(['sudo', 'nvpmodel', '-m', '0'], check=True)
    subprocess.run(['sudo', 'jetson_clocks'], check=True)

    # Configure GPU
    with open('/sys/devices/gpu.0/max_freq', 'w') as f:
        f.write('921600000')  # Set max GPU frequency

    # Configure CPU governor
    for cpu in range(6):  # Xavier NX has 6 cores
        with open(f'/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_governor', 'w') as f:
            f.write('performance')

    print("Jetson performance optimizations applied")

def check_jetson_status():
    """Check Jetson system status"""
    result = subprocess.run(['nvpmodel', '-q'], capture_output=True, text=True)
    print(f"NVPModel Status: {result.stdout}")

    result = subprocess.run(['jetson_clocks', '--show'], capture_output=True, text=True)
    print(f"Clock Status: {result.stdout}")

# Run optimizations
optimize_jetson_performance()
check_jetson_status()
```

## Deployment and Containerization

### Docker Setup for Jetson

```bash
# Install Docker for Jetson
curl -sSL https://get.docker.com/ | sh
sudo usermod -aG docker $USER

# Install NVIDIA Container Runtime
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Test with NVIDIA container
docker run --rm --gpus all -it nvcr.io/nvidia/tensorrt:23.10-py3
```

### Containerized Robotics Applications

```dockerfile
# Dockerfile for robotics application on Jetson
FROM nvcr.io/nvidia/jetson-ml:r35.4.1

# Install ROS2
RUN apt update && apt install -y \
    ros-humble-ros-base \
    python3-rosdep \
    python3-colcon-common-extensions

# Install additional packages
RUN apt install -y \
    ros-humble-navigation2 \
    ros-humble-robot-localization \
    ros-humble-usb-cam

# Set up workspace
WORKDIR /workspace
COPY . /workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Set up environment
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

CMD ["bash"]
```

## Troubleshooting Common Issues

### Memory Management Issues

```bash
# Monitor memory usage
free -h
cat /proc/meminfo

# Clear system cache to free memory
sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches'

# Check swap usage
swapon --show
```

### GPU Utilization Issues

```bash
# Check GPU status
sudo tegrastats --interval 1000  # 1 second interval

# Reset GPU if frozen
sudo pkill -f jetson_clocks
sudo jetson_clocks
```

### Thermal Throttling

```bash
# Check thermal zones
cat /sys/class/thermal/thermal_zone*/type
cat /sys/class/thermal/thermal_zone*/temp

# Monitor for throttling
sudo tegrastats --verbose
```

## Practical Exercise

Set up a complete Jetson robotics development environment:

1. Configure Jetson device with optimal performance settings
2. Install ROS2 and essential robotics packages
3. Set up sensor integration (camera and/or LiDAR)
4. Deploy a simple AI model for inference
5. Test with a basic navigation stack
6. Optimize for power consumption and thermal management

## Summary

Jetson platforms provide an excellent balance of computational power and energy efficiency for edge robotics applications. Proper configuration and optimization enable sophisticated AI inference and real-time sensor processing at the edge, making them ideal for mobile and embedded robotics systems.

## Next Steps

Continue with Realsense sensor setup, which provides high-quality depth sensing capabilities that integrate well with both Jetson platforms and RTX workstations for various robotics applications.
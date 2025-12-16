---
title: "Sensor Simulation: LiDAR, IMU, Depth Cameras"
description: "Understanding how to simulate various sensors in Gazebo for humanoid robots"
sidebar_label: "Lesson 5.3 - Sensor Simulation: LiDAR, IMU, Depth Cameras"
---

import LessonHeader from '@site/src/components/LessonHeader';
import CalloutBlock from '@site/src/components/CalloutBlock';
import QuizBlock from '@site/src/components/QuizBlock';
import AIChatPanel from '@site/src/components/AIChatPanel';
import Breadcrumb from '@site/src/components/Breadcrumb';

<Breadcrumb items={[
  { label: 'Chapters', href: '/docs/chapters/gazebo-physics-simulation' },
  { label: 'Gazebo Physics Simulation', href: '/docs/chapters/gazebo-physics-simulation' },
  { label: 'Sensor Simulation: LiDAR, IMU, Depth Cameras' }
]} />

<LessonHeader
  title="Sensor Simulation: LiDAR, IMU, Depth Cameras"
  subtitle="Understanding how to simulate various sensors in Gazebo for humanoid robots"
  chapter="5"
  lessonNumber="5.3"
  progress={75}
/>

# Sensor Simulation: LiDAR, IMU, Depth Cameras

## Learning Objectives

After completing this lesson, you will be able to:
- Configure and simulate LiDAR sensors in Gazebo for humanoid robots
- Implement realistic IMU sensor simulation with noise models
- Set up depth camera simulation for 3D perception
- Integrate sensor data with ROS 2 for perception pipelines

## Introduction

Sensor simulation is a critical component of realistic robot simulation in Gazebo. For humanoid robots, accurate simulation of sensors like LiDAR, IMU, and depth cameras is essential for developing and testing perception, navigation, and control algorithms. These sensors provide the robot with information about its environment and state, enabling autonomous behavior in simulated environments that closely match real-world conditions.

## Core Concepts

### LiDAR Simulation
LiDAR sensors in Gazebo simulate laser range finders that provide 2D or 3D distance measurements. The simulation includes beam properties, resolution, range limits, and noise models that replicate real sensor characteristics.

### IMU Simulation
Inertial Measurement Unit (IMU) simulation provides measurements of linear acceleration and angular velocity with realistic noise models and bias characteristics that match physical IMU sensors.

### Depth Camera Simulation
Depth cameras simulate RGB-D sensors that provide both color images and depth information, essential for 3D perception and mapping tasks in humanoid robotics.

## Mental Models

### Sensor Fidelity vs Performance
Thinking about the trade-offs between sensor simulation accuracy and computational performance, understanding when to prioritize realistic noise models versus simulation speed.

### Multi-Sensor Integration
Understanding how different sensor types complement each other in a humanoid robot's perception system, with each sensor providing unique information for robust operation.

## Code Examples

### Example 1: LiDAR Sensor Configuration
Configuring 2D and 3D LiDAR sensors for humanoid robot simulation:

```xml
<?xml version="1.0"?>
<robot name="humanoid_with_lidar" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include basic robot structure -->
  <xacro:include filename="$(find my_robot_description)/urdf/basic_humanoid.urdf.xacro"/>

  <!-- 2D LiDAR on the head -->
  <joint name="head_lidar_joint" type="fixed">
    <parent link="head"/>
    <child link="head_lidar_link"/>
    <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="head_lidar_link">
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <gazebo reference="head_lidar_link">
    <sensor name="head_lidar_2d" type="ray">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>  <!-- -π -->
            <max_angle>3.14159</max_angle>   <!-- π -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_2d_plugin" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/head_lidar</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>head_lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- 3D LiDAR (HDL-32E style) on top of head -->
  <joint name="top_lidar_joint" type="fixed">
    <parent link="head"/>
    <child link="top_lidar_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="top_lidar_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <gazebo reference="top_lidar_link">
    <sensor name="head_lidar_3d" type="ray">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>2120</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
          <vertical>
            <samples>32</samples>
            <resolution>1</resolution>
            <min_angle>-0.5236</min_angle>  <!-- -30 degrees -->
            <max_angle>0.1745</max_angle>   <!-- 10 degrees -->
          </vertical>
        </scan>
        <range>
          <min>0.1</min>
          <max>120.0</max>
          <resolution>0.001</resolution>
        </range>
      </ray>
      <plugin name="lidar_3d_plugin" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/top_lidar</namespace>
          <remapping>~/out:=points</remapping>
        </ros>
        <output_type>sensor_msgs/PointCloud2</output_type>
        <frame_name>top_lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Additional LiDAR on torso for close-range obstacle detection -->
  <joint name="torso_lidar_joint" type="fixed">
    <parent link="torso"/>
    <child link="torso_lidar_link"/>
    <origin xyz="0.15 0 -0.1" rpy="0 0 0"/>
  </joint>

  <link name="torso_lidar_link">
    <inertial>
      <mass value="0.15"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <gazebo reference="torso_lidar_link">
    <sensor name="torso_lidar" type="ray">
      <always_on>true</always_on>
      <update_rate>20</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.05</min>
          <max>5.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="torso_lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/torso_lidar</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>torso_lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### Example 2: IMU Sensor Configuration
Setting up realistic IMU sensors with proper noise models:

```xml
<?xml version="1.0"?>
<robot name="humanoid_with_imu" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include basic robot structure -->
  <xacro:include filename="$(find my_robot_description)/urdf/basic_humanoid.urdf.xacro"/>

  <!-- IMU in the torso (center of mass location) -->
  <gazebo reference="torso">
    <sensor name="torso_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <!-- Angular velocity noise -->
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>  <!-- 0.011 deg/s (1 sigma) -->
              <bias_mean>0.00001</bias_mean>
              <bias_stddev>0.000001</bias_stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
              <bias_mean>0.00001</bias_mean>
              <bias_stddev>0.000001</bias_stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
              <bias_mean>0.00001</bias_mean>
              <bias_stddev>0.000001</bias_stddev>
            </noise>
          </z>
        </angular_velocity>

        <!-- Linear acceleration noise -->
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>  <!-- 0.017 m/s² (1 sigma) -->
              <bias_mean>0.01</bias_mean>
              <bias_stddev>0.001</bias_stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
              <bias_mean>0.01</bias_mean>
              <bias_stddev>0.001</bias_stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
              <bias_mean>0.01</bias_mean>
              <bias_stddev>0.001</bias_stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>

      <plugin name="torso_imu_plugin" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/torso_imu</namespace>
          <remapping>~/out:=data</remapping>
        </ros>
        <frame_name>torso</frame_name>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Additional IMU in the head for head orientation tracking -->
  <gazebo reference="head">
    <sensor name="head_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.5e-4</stddev>
              <bias_mean>0.000005</bias_mean>
              <bias_stddev>0.0000005</bias_stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.5e-4</stddev>
              <bias_mean>0.000005</bias_mean>
              <bias_stddev>0.0000005</bias_stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.5e-4</stddev>
              <bias_mean>0.000005</bias_mean>
              <bias_stddev>0.0000005</bias_stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.5e-2</stddev>
              <bias_mean>0.005</bias_mean>
              <bias_stddev>0.0005</bias_stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.5e-2</stddev>
              <bias_mean>0.005</bias_mean>
              <bias_stddev>0.0005</bias_stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.5e-2</stddev>
              <bias_mean>0.005</bias_mean>
              <bias_stddev>0.0005</bias_stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>

      <plugin name="head_imu_plugin" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/head_imu</namespace>
          <remapping>~/out:=data</remapping>
        </ros>
        <frame_name>head</frame_name>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU in left foot for contact detection and balance -->
  <gazebo reference="left_foot">
    <sensor name="left_foot_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>200</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>5e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>5e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>5e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2.5e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2.5e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2.5e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>

      <plugin name="left_foot_imu_plugin" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/left_foot_imu</namespace>
          <remapping>~/out:=data</remapping>
        </ros>
        <frame_name>left_foot</frame_name>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### Example 3: Depth Camera Configuration
Setting up depth camera sensors for 3D perception:

```xml
<?xml version="1.0"?>
<robot name="humanoid_with_depth_camera" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include basic robot structure -->
  <xacro:include filename="$(find my_robot_description)/urdf/basic_humanoid.urdf.xacro"/>

  <!-- Head-mounted RGB-D camera -->
  <joint name="head_camera_joint" type="fixed">
    <parent link="head"/>
    <child link="head_camera_link"/>
    <origin xyz="0.05 0 0.02" rpy="0 0 0"/>
  </joint>

  <link name="head_camera_link">
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>
    </inertial>
  </link>

  <gazebo reference="head_camera_link">
    <sensor name="head_rgbd_camera" type="depth">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="head_camera_plugin" filename="libgazebo_ros_openni_kinect.so">
        <ros>
          <namespace>/head_camera</namespace>
          <remapping>image_raw:=image_color</remapping>
          <remapping>depth/image_raw:=image_depth</remapping>
          <remapping>depth/camera_info:=camera_info_depth</remapping>
        </ros>
        <camera_name>head_camera</camera_name>
        <frame_name>head_camera_link</frame_name>
        <baseline>0.1</baseline>
        <distortion_k1>0.0</distortion_k1>
        <distortion_k2>0.0</distortion_k2>
        <distortion_k3>0.0</distortion_k3>
        <distortion_t1>0.0</distortion_t1>
        <distortion_t2>0.0</distortion_t2>
        <point_cloud_cutoff>0.1</point_cloud_cutoff>
        <point_cloud_cutoff_max>10.0</point_cloud_cutoff_max>
        <CxPrime>0</CxPrime>
        <Cx>320.5</Cx>
        <Cy>240.5</Cy>
        <focal_length>320.0</focal_length>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Chest-mounted wide-angle depth camera -->
  <joint name="chest_camera_joint" type="fixed">
    <parent link="torso"/>
    <child link="chest_camera_link"/>
    <origin xyz="0.1 0 0.1" rpy="0 -0.1 0"/>  <!-- Slight downward angle -->
  </joint>

  <link name="chest_camera_link">
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>
    </inertial>
  </link>

  <gazebo reference="chest_camera_link">
    <sensor name="chest_depth_camera" type="depth">
      <always_on>true</always_on>
      <update_rate>15</update_rate>
      <camera name="chest_camera">
        <horizontal_fov>1.571</horizontal_fov> <!-- 90 degrees for wider view -->
        <image>
          <width>320</width>
          <height>240</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.2</near>
          <far>8.0</far>
        </clip>
      </camera>
      <plugin name="chest_camera_plugin" filename="libgazebo_ros_openni_kinect.so">
        <ros>
          <namespace>/chest_camera</namespace>
          <remapping>image_raw:=image_color</remapping>
          <remapping>depth/image_raw:=image_depth</remapping>
        </ros>
        <camera_name>chest_camera</camera_name>
        <frame_name>chest_camera_link</frame_name>
        <baseline>0.075</baseline>
        <focal_length>160.0</focal_length>
        <point_cloud_cutoff>0.2</point_cloud_cutoff>
        <point_cloud_cutoff_max>8.0</point_cloud_cutoff_max>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Stereo camera setup for better depth estimation -->
  <joint name="stereo_left_joint" type="fixed">
    <parent link="head"/>
    <child link="stereo_left_camera_link"/>
    <origin xyz="0.06 0.03 0.02" rpy="0 0 0"/>
  </joint>

  <joint name="stereo_right_joint" type="fixed">
    <parent link="head"/>
    <child link="stereo_right_camera_link"/>
    <origin xyz="0.06 -0.03 0.02" rpy="0 0 0"/>
  </joint>

  <link name="stereo_left_camera_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <link name="stereo_right_camera_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <gazebo reference="stereo_left_camera_link">
    <sensor name="stereo_left_camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera name="stereo_left">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="stereo_left_plugin" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/stereo</namespace>
          <remapping>image_raw:=left/image_raw</remapping>
          <remapping>camera_info:=left/camera_info</remapping>
        </ros>
        <camera_name>stereo_left</camera_name>
        <frame_name>stereo_left_camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="stereo_right_camera_link">
    <sensor name="stereo_right_camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera name="stereo_right">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="stereo_right_plugin" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/stereo</namespace>
          <remapping>image_raw:=right/image_raw</remapping>
          <remapping>camera_info:=right/camera_info</remapping>
        </ros>
        <camera_name>stereo_right</camera_name>
        <frame_name>stereo_right_camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### Example 4: Python Sensor Data Processing
Processing sensor data from simulated sensors:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import cv2
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import message_filters

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Create subscribers for different sensor types
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/head_lidar/scan',
            self.lidar_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/torso_imu/data',
            self.imu_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/head_camera/image_color',
            self.camera_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/head_camera/image_depth',
            self.depth_callback,
            10
        )

        # Publisher for processed data
        self.obstacle_pub = self.create_publisher(
            LaserScan,
            '/processed_obstacles',
            10
        )

        # CV Bridge for image processing
        self.cv_bridge = CvBridge()

        # Data storage
        self.latest_imu_data = None
        self.latest_image = None
        self.latest_depth = None

        self.get_logger().info('Sensor processor initialized')

    def lidar_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        # Convert to numpy array for processing
        ranges = np.array(msg.ranges)

        # Filter out invalid ranges
        valid_ranges = ranges[np.isfinite(ranges)]

        # Detect obstacles within 1 meter
        obstacle_ranges = valid_ranges[valid_ranges < 1.0]

        if len(obstacle_ranges) > 0:
            # Calculate obstacle density
            obstacle_density = len(obstacle_ranges) / len(valid_ranges)

            if obstacle_density > 0.1:  # More than 10% of readings are obstacles
                self.get_logger().warn(f'High obstacle density detected: {obstacle_density:.2f}')

                # Publish processed obstacle information
                processed_msg = LaserScan()
                processed_msg.header = msg.header
                processed_msg.angle_min = msg.angle_min
                processed_msg.angle_max = msg.angle_max
                processed_msg.angle_increment = msg.angle_increment
                processed_msg.time_increment = msg.time_increment
                processed_msg.scan_time = msg.scan_time
                processed_msg.range_min = msg.range_min
                processed_msg.range_max = msg.range_max
                processed_msg.ranges = [r if r < 1.0 else float('inf') for r in msg.ranges]

                self.obstacle_pub.publish(processed_msg)

    def imu_callback(self, msg):
        """Process IMU data for balance and orientation"""
        # Extract orientation (using quaternion)
        orientation = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])

        # Extract angular velocity
        angular_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Extract linear acceleration
        linear_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Calculate roll, pitch, yaw from quaternion
        qw, qx, qy, qz = orientation
        roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = np.arcsin(2*(qw*qy - qz*qx))
        yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

        # Store for other processes
        self.latest_imu_data = {
            'orientation': (roll, pitch, yaw),
            'angular_velocity': angular_vel,
            'linear_acceleration': linear_acc
        }

        # Check for balance issues
        if abs(pitch) > 0.5 or abs(roll) > 0.5:  # 0.5 rad = ~28.6 degrees
            self.get_logger().warn(f'Potential balance issue: pitch={pitch:.2f}, roll={roll:.2f}')

    def camera_callback(self, msg):
        """Process camera image"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Store for other processes
            self.latest_image = cv_image

            # Example: Detect edges using Canny
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Count edges as a simple scene complexity metric
            edge_density = np.sum(edges > 0) / (edges.shape[0] * edges.shape[1])

            if edge_density > 0.1:  # More than 10% of pixels are edges
                self.get_logger().info(f'High edge density detected: {edge_density:.2f}')

        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def depth_callback(self, msg):
        """Process depth image"""
        try:
            # Convert ROS Image message to OpenCV image
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

            # Store for other processes
            self.latest_depth = depth_image

            # Calculate distance statistics
            valid_depths = depth_image[np.isfinite(depth_image)]
            if len(valid_depths) > 0:
                avg_depth = np.mean(valid_depths)
                min_depth = np.min(valid_depths)
                max_depth = np.max(valid_depths)

                self.get_logger().debug(f'Depth stats - Avg: {avg_depth:.2f}, Min: {min_depth:.2f}, Max: {max_depth:.2f}')

                # Detect nearby objects
                close_objects = valid_depths[valid_depths < 1.0]
                if len(close_objects) > 0:
                    self.get_logger().info(f'Detected {len(close_objects)} close objects (< 1m)')

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def get_robot_state_estimate(self):
        """Combine sensor data to estimate robot state"""
        state = {}

        if self.latest_imu_data:
            state['orientation'] = self.latest_imu_data['orientation']
            state['angular_velocity'] = self.latest_imu_data['angular_velocity']
            state['linear_acceleration'] = self.latest_imu_data['linear_acceleration']

        if self.latest_image is not None:
            state['image_shape'] = self.latest_image.shape
            state['has_image'] = True

        if self.latest_depth is not None:
            state['depth_range'] = (np.min(self.latest_depth), np.max(self.latest_depth))
            state['has_depth'] = True

        return state

def main(args=None):
    rclpy.init(args=args)

    sensor_processor = SensorProcessor()

    # Example usage of sensor fusion
    timer = sensor_processor.create_timer(1.0, lambda:
        sensor_processor.get_logger().info(f'Robot state: {sensor_processor.get_robot_state_estimate()}'))

    try:
        rclpy.spin(sensor_processor)
    except KeyboardInterrupt:
        sensor_processor.get_logger().info('Shutting down sensor processor')
    finally:
        sensor_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Simulation Exercises

### Exercise 1: LiDAR Sensor Integration
- **Objective**: Integrate LiDAR sensors into a humanoid robot model
- **Requirements**: Gazebo, ROS 2, robot model
- **Steps**:
  1. Add 2D LiDAR to the robot's head
  2. Configure appropriate scan parameters (range, resolution, update rate)
  3. Test obstacle detection in various environments
  4. Validate sensor data quality and noise characteristics
- **Expected Outcome**: Working LiDAR sensor with realistic data output

### Exercise 2: IMU Sensor Calibration
- **Objective**: Configure and validate IMU sensors with realistic noise
- **Requirements**: Gazebo, ROS 2, IMU sensor configuration
- **Steps**:
  1. Add IMU to robot torso and other strategic locations
  2. Configure noise parameters based on real IMU specifications
  3. Test under static and dynamic conditions
  4. Validate orientation and acceleration measurements
- **Expected Outcome**: Realistic IMU data suitable for balance and navigation

### Exercise 3: Depth Camera Perception Pipeline
- **Objective**: Implement a complete depth camera perception pipeline
- **Requirements**: Gazebo, ROS 2, OpenCV, perception packages
- **Steps**:
  1. Configure depth camera with appropriate parameters
  2. Implement image processing for obstacle detection
  3. Create 3D point cloud processing
  4. Test perception in various lighting conditions
- **Expected Outcome**: Functional perception pipeline using simulated depth data

## Summary

Sensor simulation in Gazebo provides realistic data for developing and testing humanoid robot perception systems. LiDAR, IMU, and depth cameras each provide unique information that, when properly configured with realistic noise models, enable the development of robust perception and control algorithms. The integration of multiple sensor types creates a comprehensive perception system that closely matches real-world robot capabilities.

## Key Terms

- **LiDAR**: Light Detection and Ranging sensor that measures distances using laser pulses
- **IMU**: Inertial Measurement Unit that measures acceleration and angular velocity
- **Depth Camera**: Camera that captures both color and depth information per pixel
- **Point Cloud**: Set of 3D points representing a 3D shape or object
- **Sensor Noise**: Random variations in sensor measurements that model real sensor imperfections
- **Update Rate**: Frequency at which sensor data is published
- **Field of View**: Angular extent of the observable world that is seen at any given moment
- **Sensor Fusion**: Combining data from multiple sensors to improve accuracy and robustness

## Next Steps

Continue to Chapter 6: "Unity Visualization & Interaction" to explore how to integrate Unity for advanced visualization and human-robot interaction.
---
title: Setting up Gazebo
description: Understanding how to set up and configure Gazebo for robot simulation
sidebar_label: Lesson 5.1 - Setting up Gazebo
---

# Setting up Gazebo

## Learning Objectives

After completing this lesson, you will be able to:
- Install and configure Gazebo for robot simulation
- Launch Gazebo with custom environments and robot models
- Configure physics properties and simulation parameters
- Integrate Gazebo with ROS 2 for robot simulation

## Introduction

Gazebo is a powerful 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It's widely used in robotics research and development for testing algorithms, validating robot designs, and training AI systems. For humanoid robotics, Gazebo provides an essential platform for testing complex behaviors in a safe, controlled environment before deploying to real robots.

## Core Concepts

### Physics Simulation Engine
Gazebo uses physics engines like ODE (Open Dynamics Engine), Bullet, or DART to simulate realistic physical interactions including collisions, friction, gravity, and joint dynamics.

### Environment Modeling
Gazebo allows for the creation of complex environments with static and dynamic objects, lighting conditions, and sensor models that closely match real-world conditions.

### Sensor Simulation
Gazebo provides realistic simulation of various sensors including cameras, LIDAR, IMU, force/torque sensors, and GPS with configurable noise models.

## Mental Models

### Simulation as Testing Ground
Thinking of Gazebo as a safe, repeatable testing environment where robot behaviors can be validated before real-world deployment, reducing risks and development time.

### Physics Fidelity vs Performance Trade-off
Understanding the balance between simulation accuracy and computational performance, adjusting parameters based on the specific requirements of the simulation task.

## Code Examples

### Example 1: Basic Gazebo Launch with Robot
Creating a launch file to spawn a robot in Gazebo:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = LaunchConfiguration('robot_name', default='humanoid_robot')
    world_file = LaunchConfiguration('world', default='empty.world')

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0
        }]
    )

    # Gazebo server launch
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items()
    )

    # Gazebo client launch
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='humanoid_robot',
            description='Name of the robot to spawn'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty.world',
            description='Choose one of the world files from `/gazebo_ros/worlds`'
        ),
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_robot
    ])
```

### Example 2: Custom World File
Creating a custom Gazebo world with specific environment:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_test_world">
    <!-- Include the default sun and atmosphere -->
    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Custom floor with texture -->
    <model name="custom_floor">
      <static>true</static>
      <link name="floor_link">
        <collision name="floor_collision">
          <geometry>
            <box>
              <size>20 20 0.1</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <kp>1e6</kp>
                <kd>100</kd>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="floor_visual">
          <geometry>
            <box>
              <size>20 20 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacle for testing -->
    <model name="obstacle_box">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="obstacle_link">
        <collision name="obstacle_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="obstacle_visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.4 0.1 1</ambient>
            <diffuse>0.9 0.5 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Ramp for testing walking -->
    <model name="ramp">
      <pose>-3 0 0.2 0 0.2 0</pose>
      <link name="ramp_link">
        <collision name="ramp_collision">
          <geometry>
            <mesh>
              <uri>file://ramp.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="ramp_visual">
          <geometry>
            <mesh>
              <uri>file://ramp.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <inertial>
          <mass>100.0</mass>
          <inertia>
            <ixx>10.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>10.0</iyy>
            <iyz>0.0</iyz>
            <izz>10.0</izz>
          </inertia>
        </inertial>
      </link>
      <static>true</static>
    </model>

  </world>
</sdf>
```

### Example 3: Gazebo Plugin Configuration
Configuring Gazebo plugins for enhanced simulation:

```xml
<?xml version="1.0"?>
<robot name="humanoid_with_gazebo_plugins" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include basic robot structure -->
  <xacro:include filename="$(find my_robot_description)/urdf/basic_humanoid.urdf.xacro"/>

  <!-- Gazebo control plugin for ros2_control -->
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find my_robot_bringup)/config/humanoid_controllers.yaml</parameters>
      <robot_namespace>/</robot_namespace>
    </plugin>
  </gazebo>

  <!-- Gazebo ROS IMU plugin -->
  <gazebo reference="torso">
    <sensor name="torso_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/torso_imu</namespace>
          <remapping>~/out:=data</remapping>
        </ros>
        <frame_name>torso</frame_name>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Gazebo camera plugin -->
  <gazebo reference="head_camera_link">
    <sensor name="head_camera" type="camera">
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/head_camera</namespace>
          <remapping>image_raw:=image</remapping>
          <remapping>camera_info:=camera_info</remapping>
        </ros>
        <frame_name>head_camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Contact sensor for feet -->
  <gazebo reference="left_foot">
    <sensor name="left_foot_contact" type="contact">
      <always_on>true</always_on>
      <update_rate>1000</update_rate>
      <contact>
        <collision>left_foot_collision</collision>
      </contact>
      <plugin name="left_foot_contact_plugin" filename="libgazebo_ros_bumper.so">
        <ros>
          <namespace>/left_foot</namespace>
          <remapping>bumper_states:=contact</remapping>
        </ros>
        <frame_name>left_foot</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="right_foot">
    <sensor name="right_foot_contact" type="contact">
      <always_on>true</always_on>
      <update_rate>1000</update_rate>
      <contact>
        <collision>right_foot_collision</collision>
      </contact>
      <plugin name="right_foot_contact_plugin" filename="libgazebo_ros_bumper.so">
        <ros>
          <namespace>/right_foot</namespace>
          <remapping>bumper_states:=contact</remapping>
        </ros>
        <frame_name>right_foot</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Simulation Exercises

### Exercise 1: Gazebo Installation and Basic Setup
- **Objective**: Install Gazebo and run a basic simulation
- **Requirements**: Ubuntu 22.04, ROS 2 Humble, Gazebo Garden
- **Steps**:
  1. Install Gazebo Garden following official instructions
  2. Launch Gazebo with the default empty world
  3. Add a simple robot model to the simulation
  4. Test basic interaction with the simulation
- **Expected Outcome**: Working Gazebo installation with basic robot simulation

### Exercise 2: Custom Environment Creation
- **Objective**: Create a custom Gazebo world for humanoid testing
- **Requirements**: Gazebo, XML knowledge
- **Steps**:
  1. Create a new world file with obstacles
  2. Configure physics properties appropriately
  3. Add textured surfaces and lighting
  4. Test the world with a robot model
- **Expected Outcome**: Custom environment suitable for humanoid robot testing

### Exercise 3: Robot Integration with Gazebo
- **Objective**: Integrate a humanoid robot model with Gazebo
- **Requirements**: URDF model, Gazebo, ROS 2
- **Steps**:
  1. Add Gazebo plugins to your robot URDF
  2. Configure sensors (IMU, camera, contact sensors)
  3. Set up ros2_control integration
  4. Test robot control and sensing in simulation
- **Expected Outcome**: Fully integrated robot model with working sensors and control

## Summary

Setting up Gazebo for humanoid robot simulation involves configuring the physics engine, creating appropriate environments, and integrating robot models with sensors and control systems. Gazebo provides a powerful platform for testing robot behaviors in a safe, repeatable environment before deployment to real hardware. Proper configuration of physics parameters, sensors, and plugins is essential for realistic simulation results.

## Key Terms

- **Gazebo**: 3D simulation environment for robotics with physics simulation
- **SDF**: Simulation Description Format, Gazebo's native model format
- **Physics Engine**: Software that simulates physical interactions (ODE, Bullet, DART)
- **Gazebo Plugin**: Extensions that provide additional functionality to Gazebo
- **Contact Sensor**: Sensor that detects when objects make contact
- **Simulation Time**: Time that progresses in the simulated world
- **Real-time Factor**: Ratio of simulation time to real time
- **Collision Detection**: System that identifies when objects intersect

## Next Steps

Continue to Lesson 5.2: "Physics, Gravity, and Collisions" to explore the physics simulation aspects of Gazebo in detail.
---
title: Launch Files and Parameters
description: Understanding how to configure and launch ROS 2 systems
sidebar_label: Lesson 3.2 - Launch Files and Parameters
---

import LessonHeader from '@site/src/components/LessonHeader';
import CalloutBlock from '@site/src/components/CalloutBlock';
import QuizBlock from '@site/src/components/QuizBlock';
import AIChatPanel from '@site/src/components/AIChatPanel';
import Breadcrumb from '@site/src/components/Breadcrumb';

<Breadcrumb items={[
  { label: 'Chapters', href: '/docs/chapters/ros2-architecture' },
  { label: 'ROS 2 Architecture', href: '/docs/chapters/ros2-architecture' },
  { label: 'Launch Files and Parameters' }
]} />

<LessonHeader
  title="Launch Files and Parameters"
  subtitle="Understanding how to configure and launch ROS 2 systems"
  chapter="3"
  lessonNumber="3.2"
  progress={50}
/>

# Launch Files and Parameters

![ROS 2 Communication Patterns](/img/ros2-communication-patterns.jpeg)

## Learning Objectives

After completing this lesson, you will be able to:
- Create and configure launch files for complex ROS 2 systems
- Define and use parameters in ROS 2 nodes
- Organize robot systems using launch file composition
- Manage system configuration through parameter files

## Introduction

Launch files in ROS 2 provide a declarative way to start multiple nodes with specific configurations simultaneously. They allow for complex robot systems to be launched with a single command, managing node parameters, remappings, and lifecycle management. Parameters provide a flexible way to configure node behavior without recompilation.

## Core Concepts

### Launch Files
Launch files define how a ROS 2 system should be started, including which nodes to launch, their parameters, remappings, and other configurations. They are typically written in Python using the launch library.

### Parameters
Parameters in ROS 2 allow nodes to be configured externally. They can be set at launch time, changed dynamically during runtime, and stored in configuration files for different deployment scenarios.

### Launch Compositions
Launch files can include other launch files, enabling hierarchical organization of complex systems. This promotes reusability and modularity in robot system design.

## Mental Models

### System Configuration as Code
Launch files treat the entire robot system configuration as code, allowing for version control, testing, and reproducible deployments across different environments.

### Parameter Hierarchy
ROS 2 follows a parameter hierarchy where parameters can be set at different levels (system-wide, node-specific, etc.) with more specific settings overriding general ones.

## Code Examples

### Example 1: Basic Launch File
Creating a launch file for a simple robot system:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        # Robot state publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_frequency': 50.0
            }],
            output='screen'
        ),

        # Joint state publisher node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rate': 50
            }],
            output='screen'
        )
    ])
```

### Example 2: Parameter Configuration
Using YAML parameter files to configure nodes:

```yaml
# config/robot_params.yaml
/**:
  ros__parameters:
    use_sim_time: false
    control_frequency: 100.0
    safety_limits:
      max_velocity: 1.0
      max_acceleration: 2.0

controller_manager:
  ros__parameters:
    update_rate: 100
    use_sim_time: false

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
    state_publish_rate: 50
    action_monitor_rate: 20
```

### Example 3: Advanced Launch Configuration
Creating a launch file with conditional logic and node composition:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # Get package directories
    pkg_path = get_package_share_directory('my_robot_bringup')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # Robot state publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[os.path.join(pkg_path, 'config', 'robot_params.yaml')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Sensor processing node
    sensor_processing_cmd = Node(
        package='my_robot_perception',
        executable='sensor_processing',
        name='sensor_processing',
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(pkg_path, 'config', 'sensors.yaml')
        ],
        output='screen'
    )

    # Control node
    control_node_cmd = Node(
        package='my_robot_control',
        executable='control_node',
        name='control_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(pkg_path, 'config', 'control.yaml')
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_rviz_cmd,
        robot_state_publisher_cmd,
        sensor_processing_cmd,
        control_node_cmd,
    ])
```

## Simulation Exercises

### Exercise 1: Launch File Creation
- **Objective**: Create a launch file for a mobile robot system
- **Requirements**: ROS 2 Foxy/Humble, Python
- **Steps**:
  1. Create a launch file that starts a robot state publisher
  2. Add a joint state publisher with custom parameters
  3. Include a navigation stack launch file
  4. Test the launch file with different parameter configurations
- **Expected Outcome**: Understanding of launch file structure and parameter management

### Exercise 2: Parameter Management
- **Objective**: Implement parameter configuration for a robot controller
- **Requirements**: ROS 2 Foxy/Humble, Python/YAML
- **Steps**:
  1. Create a YAML parameter file for controller settings
  2. Modify parameters during runtime using ros2 param commands
  3. Create multiple parameter configurations for different scenarios
  4. Test parameter loading and validation
- **Expected Outcome**: Understanding of parameter management in ROS 2

### Exercise 3: Launch Composition
- **Objective**: Create a hierarchical launch system
- **Requirements**: ROS 2 Foxy/Humble, Python
- **Steps**:
  1. Create separate launch files for different robot subsystems
  2. Create a master launch file that includes all subsystems
  3. Implement conditional launching based on arguments
  4. Test the composition with different configurations
- **Expected Outcome**: Understanding of launch file composition and modularity

## Summary

Launch files and parameters provide powerful mechanisms for configuring and launching complex ROS 2 systems. Launch files enable reproducible system startup with specific configurations, while parameters allow for runtime configuration without code changes. Understanding these concepts is essential for building maintainable and configurable robot systems.

## Key Terms

- **Launch File**: A Python file that defines how to start a ROS 2 system
- **Parameter**: Configurable values that control node behavior
- **Launch Configuration**: Arguments passed to launch files to customize behavior
- **Parameter Server**: Component that manages parameters in ROS 2
- **Launch Composition**: The ability to include launch files within other launch files
- **Node Remapping**: Changing node names, topic names, or service names at launch

## Next Steps

Continue to Lesson 3.3 to explore additional ROS 2 architecture concepts and best practices for system design.
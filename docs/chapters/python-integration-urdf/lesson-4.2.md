---
title: Creating and Using URDF for Humanoids
description: Understanding Unified Robot Description Format (URDF) for humanoid robot modeling
sidebar_label: Lesson 4.2 - Creating and Using URDF for Humanoids
---

import LessonHeader from '@site/src/components/LessonHeader';
import CalloutBlock from '@site/src/components/CalloutBlock';
import QuizBlock from '@site/src/components/QuizBlock';
import AIChatPanel from '@site/src/components/AIChatPanel';
import Breadcrumb from '@site/src/components/Breadcrumb';

<Breadcrumb items={[
  { label: 'Chapters', href: '/docs/chapters/python-integration-urdf' },
  { label: 'Python Integration & URDF', href: '/docs/chapters/python-integration-urdf' },
  { label: 'Creating and Using URDF for Humanoids' }
]} />

<LessonHeader
  title="Creating and Using URDF for Humanoids"
  subtitle="Understanding Unified Robot Description Format (URDF) for humanoid robot modeling"
  chapter="4"
  lessonNumber="4.2"
  progress={50}
/>

# Creating and Using URDF for Humanoids

![URDF Joints Relationship](/img/urdf-joints-relationship.jpeg)

## Learning Objectives

After completing this lesson, you will be able to:
- Create URDF files for complex humanoid robot models
- Define joints, links, and physical properties for robot kinematics
- Use Xacro macros to simplify complex robot descriptions
- Validate and visualize humanoid robot models in RViz

## Introduction

Unified Robot Description Format (URDF) is an XML-based format used to describe robot models in ROS. For humanoid robots, URDF provides the foundation for kinematic and dynamic simulation, motion planning, and visualization. Understanding URDF is crucial for creating accurate robot models that can be used in simulation and real-world applications.

## Core Concepts

### Links and Joints
URDF models consist of links (rigid bodies) connected by joints (constraints). Links define the physical properties of robot parts, while joints define how they can move relative to each other.

### Kinematic Chains
Humanoid robots typically have multiple kinematic chains (e.g., arms, legs) that start from a base link and extend to end-effectors like hands and feet.

### Inertial Properties
Accurate inertial properties (mass, center of mass, moments of inertia) are essential for dynamic simulation and control.

## Mental Models

### Robot as Assembly
Thinking of a robot as an assembly of rigid bodies connected by joints, where each component has specific physical properties and constraints.

### Hierarchical Structure
Understanding that robot models form a tree structure (with loops for closed chains) where transformations propagate from parent to child links.

## Code Examples

### Example 1: Basic Humanoid Robot URDF
Creating a simple humanoid robot model:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="2.0"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0008"/>
    </inertial>
  </link>

  <!-- Right Arm (similar to left, mirrored) -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.15 -0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="2.0"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0008"/>
    </inertial>
  </link>

</robot>
```

### Example 2: Xacro Macros for Humanoid Robot
Using Xacro to simplify complex humanoid robot definitions:

```xml
<?xml version="1.0"?>
<robot name="xacro_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_length" value="0.4" />
  <xacro:property name="arm_length" value="0.3" />
  <xacro:property name="leg_length" value="0.5" />

  <!-- Macro for creating a simple arm -->
  <xacro:macro name="simple_arm" params="side parent_link parent_joint_origin">
    <!-- Shoulder joint -->
    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${parent_joint_origin}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="50" velocity="2.0"/>
    </joint>

    <!-- Upper arm link -->
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="${arm_length}" radius="0.05"/>
        </geometry>
        <origin xyz="0 0 ${arm_length/2}" rpy="${M_PI/2} 0 0"/>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${arm_length}" radius="0.05"/>
        </geometry>
        <origin xyz="0 0 ${arm_length/2}" rpy="${M_PI/2} 0 0"/>
      </collision>
      <inertial>
        <mass value="1.5"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <!-- Elbow joint -->
    <joint name="${side}_elbow_joint" type="revolute">
      <parent link="${side}_upper_arm"/>
      <child link="${side}_lower_arm"/>
      <origin xyz="0 0 ${arm_length}" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="30" velocity="2.0"/>
    </joint>

    <!-- Lower arm link -->
    <link name="${side}_lower_arm">
      <visual>
        <geometry>
          <cylinder length="${arm_length*0.8}" radius="0.04"/>
        </geometry>
        <origin xyz="0 0 ${arm_length*0.4}" rpy="${M_PI/2} 0 0"/>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${arm_length*0.8}" radius="0.04"/>
        </geometry>
        <origin xyz="0 0 ${arm_length*0.4}" rpy="${M_PI/2} 0 0"/>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0008"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Macro for creating a simple leg -->
  <xacro:macro name="simple_leg" params="side parent_link parent_joint_origin">
    <!-- Hip joint -->
    <joint name="${side}_hip_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${side}_thigh"/>
      <origin xyz="${parent_joint_origin}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI/3}" upper="${M_PI/3}" effort="100" velocity="1.0"/>
    </joint>

    <!-- Thigh link -->
    <link name="${side}_thigh">
      <visual>
        <geometry>
          <cylinder length="${leg_length}" radius="0.06"/>
        </geometry>
        <origin xyz="0 0 ${-leg_length/2}" rpy="${M_PI/2} 0 0"/>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${leg_length}" radius="0.06"/>
        </geometry>
        <origin xyz="0 0 ${-leg_length/2}" rpy="${M_PI/2} 0 0"/>
      </collision>
      <inertial>
        <mass value="3.0"/>
        <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>

    <!-- Knee joint -->
    <joint name="${side}_knee_joint" type="revolute">
      <parent link="${side}_thigh"/>
      <child link="${side}_shin"/>
      <origin xyz="0 0 ${-leg_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="${M_PI/2}" effort="100" velocity="1.0"/>
    </joint>

    <!-- Shin link -->
    <link name="${side}_shin">
      <visual>
        <geometry>
          <cylinder length="${leg_length*0.9}" radius="0.05"/>
        </geometry>
        <origin xyz="0 0 ${-leg_length*0.45}" rpy="${M_PI/2} 0 0"/>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${leg_length*0.9}" radius="0.05"/>
        </geometry>
        <origin xyz="0 0 ${-leg_length*0.45}" rpy="${M_PI/2} 0 0"/>
      </collision>
      <inertial>
        <mass value="2.5"/>
        <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.008"/>
      </inertial>
    </link>

    <!-- Ankle joint -->
    <joint name="${side}_ankle_joint" type="revolute">
      <parent link="${side}_shin"/>
      <child link="${side}_foot"/>
      <origin xyz="0 0 ${-leg_length*0.9}" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="50" velocity="1.0"/>
    </joint>

    <!-- Foot link -->
    <link name="${side}_foot">
      <visual>
        <geometry>
          <box size="0.2 0.1 0.05"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <box size="0.2 0.1 0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.15 ${torso_length}"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 ${torso_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 ${torso_length/2}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Create arms using macro -->
  <xacro:simple_arm side="left" parent_link="torso" parent_joint_origin="0.15 0.1 0.1"/>
  <xacro:simple_arm side="right" parent_link="torso" parent_joint_origin="0.15 -0.1 0.1"/>

  <!-- Create legs using macro -->
  <xacro:simple_leg side="left" parent_link="base_link" parent_joint_origin="0.05 0.05 -0.05"/>
  <xacro:simple_leg side="right" parent_link="base_link" parent_joint_origin="0.05 -0.05 -0.05"/>

</robot>
```

### Example 3: Python Script for URDF Validation and Processing
Using Python to work with URDF files:

```python
#!/usr/bin/env python3

import xml.etree.ElementTree as ET
from xml.dom import minidom
import math

class URDFValidator:
    def __init__(self, urdf_file):
        self.urdf_file = urdf_file
        self.tree = ET.parse(urdf_file)
        self.root = self.tree.getroot()

    def validate_inertial_properties(self):
        """Validate that all links have proper inertial properties"""
        issues = []

        for link in self.root.findall('link'):
            name = link.get('name')
            inertial = link.find('inertial')

            if inertial is None:
                issues.append(f"Link '{name}' has no inertial properties")
                continue

            mass = inertial.find('mass')
            if mass is None or float(mass.get('value')) <= 0:
                issues.append(f"Link '{name}' has invalid mass")

            inertia = inertial.find('inertia')
            if inertia is None:
                issues.append(f"Link '{name}' has no inertia matrix")
            else:
                # Check if inertia values are physically plausible
                ixx = float(inertia.get('ixx', 0))
                iyy = float(inertia.get('iyy', 0))
                izz = float(inertia.get('izz', 0))

                if ixx <= 0 or iyy <= 0 or izz <= 0:
                    issues.append(f"Link '{name}' has invalid inertia values")

        return issues

    def validate_joint_limits(self):
        """Validate joint limits and types"""
        issues = []

        for joint in self.root.findall('joint'):
            name = joint.get('name')
            joint_type = joint.get('type')

            if joint_type in ['revolute', 'prismatic']:
                limit = joint.find('limit')
                if limit is None:
                    issues.append(f"Joint '{name}' has no limits (required for {joint_type})")
                else:
                    lower = float(limit.get('lower', 0))
                    upper = float(limit.get('upper', 0))

                    if lower >= upper:
                        issues.append(f"Joint '{name}' has invalid limits: lower >= upper")

        return issues

    def get_kinematic_chain(self, base_link='base_link', end_effector='right_hand'):
        """Get the kinematic chain from base to end effector"""
        # Build parent-child relationships
        parent_map = {}
        for joint in self.root.findall('joint'):
            child = joint.find('child').get('link')
            parent = joint.find('parent').get('link')
            parent_map[child] = parent

        # Find the chain
        chain = []
        current = end_effector

        while current != base_link and current in parent_map:
            chain.append(current)
            current = parent_map[current]

        if current == base_link:
            chain.append(base_link)
            chain.reverse()
            return chain
        else:
            return []  # No path found

    def print_robot_info(self):
        """Print basic robot information"""
        robot_name = self.root.get('name')
        print(f"Robot: {robot_name}")

        links = self.root.findall('link')
        joints = self.root.findall('joint')

        print(f"Links: {len(links)}")
        print(f"Joints: {len(joints)}")

        # Count different joint types
        joint_types = {}
        for joint in joints:
            jtype = joint.get('type')
            joint_types[jtype] = joint_types.get(jtype, 0) + 1

        print("Joint types:")
        for jtype, count in joint_types.items():
            print(f"  {jtype}: {count}")

def main():
    # Example usage
    print("URDF Processing Example")
    print("=" * 30)

    # Note: This would normally process an actual URDF file
    # For this example, we'll just show the structure

    print("This script would validate and process URDF files")
    print("It can:")
    print("- Validate inertial properties")
    print("- Check joint limits and constraints")
    print("- Analyze kinematic chains")
    print("- Generate kinematic models")
    print("- Export for simulation engines")

if __name__ == '__main__':
    main()
```

## Simulation Exercises

### Exercise 1: URDF Robot Creation
- **Objective**: Create a complete humanoid robot URDF model
- **Requirements**: ROS 2, URDF/Xacro knowledge
- **Steps**:
  1. Define the base link and torso
  2. Add arms with shoulder, elbow, and wrist joints
  3. Add legs with hip, knee, and ankle joints
  4. Include proper visual, collision, and inertial properties
  5. Validate the model using check_urdf tool
- **Expected Outcome**: Complete humanoid robot model that can be visualized in RViz

### Exercise 2: Xacro Macro Development
- **Objective**: Create Xacro macros to simplify robot definition
- **Requirements**: Xacro knowledge, URDF understanding
- **Steps**:
  1. Create macros for arms, legs, and other repeated components
  2. Use parameters to customize macro instances
  3. Include proper joint limits and physical properties
  4. Test the generated URDF with rviz
- **Expected Outcome**: Reusable Xacro macros that simplify complex robot definitions

### Exercise 3: Robot Model Validation
- **Objective**: Validate and debug URDF models
- **Requirements**: ROS 2, URDF tools
- **Steps**:
  1. Create a URDF with intentional errors
  2. Use check_urdf tool to identify issues
  3. Fix the errors and validate again
  4. Test kinematic chains and joint ranges
- **Expected Outcome**: Understanding of URDF validation and debugging techniques

## Summary

URDF provides a comprehensive framework for describing robot models in ROS, which is essential for humanoid robotics applications. By properly defining links, joints, and physical properties, developers can create accurate models for simulation, visualization, and control. Using Xacro macros helps manage the complexity of humanoid robots with many degrees of freedom.

## Key Terms

- **URDF**: Unified Robot Description Format, an XML-based robot modeling language
- **Link**: A rigid body in the robot model with visual, collision, and inertial properties
- **Joint**: A constraint connecting two links with specific motion characteristics
- **Xacro**: XML macro language that extends URDF with macros and parameters
- **Kinematic Chain**: A sequence of links connected by joints from base to end-effector
- **Inertial Properties**: Mass, center of mass, and moments of inertia for dynamic simulation
- **Joint Limits**: Constraints on joint position, velocity, and effort
- **Collision Geometry**: Shapes used for collision detection and physics simulation

## Next Steps

Continue to Lesson 4.3 to explore advanced URDF concepts and integration with ROS 2 systems.
---
title: Physics, Gravity, and Collisions
description: Understanding physics simulation, gravity effects, and collision handling in Gazebo
sidebar_label: Lesson 5.2 - Physics, Gravity, and Collisions
---

# Physics, Gravity, and Collisions

## Learning Objectives

After completing this lesson, you will be able to:
- Configure physics engine parameters for realistic simulation
- Understand and adjust gravity settings for different environments
- Implement proper collision detection and response mechanisms
- Optimize physics parameters for humanoid robot simulation

## Introduction

Physics simulation is the core of Gazebo's functionality, providing realistic interactions between objects in the virtual environment. For humanoid robots, accurate physics simulation is crucial for testing locomotion, manipulation, and interaction behaviors. Understanding how to configure and tune physics parameters allows for realistic simulation of robot dynamics, including gravity effects, friction, and collision responses that closely match real-world behavior.

## Core Concepts

### Physics Engine Parameters
The physics engine parameters control the accuracy and performance of the simulation, including time step size, solver iterations, and constraint handling that affect how objects interact.

### Gravity Simulation
Gravity settings determine how objects fall, interact with surfaces, and behave dynamically, which is essential for humanoid robots that must maintain balance and control their movements.

### Collision Detection
Collision detection systems identify when objects intersect and generate appropriate responses, including contact forces, friction, and bounce effects that affect robot behavior.

## Mental Models

### Physics Accuracy vs Performance
Thinking about the trade-offs between simulation accuracy and computational performance, understanding when to prioritize realism versus speed based on the simulation goals.

### Dynamic Equilibrium
Understanding how robots maintain balance and stability through the interplay of forces, torques, and control systems in the simulated physics environment.

## Code Examples

### Example 1: Physics Configuration for Humanoid Simulation
Configuring physics parameters optimized for humanoid robot simulation:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_physics_world">
    <!-- Include default models -->
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Physics engine with humanoid-optimized parameters -->
    <physics type="ode">
      <!-- Time step - smaller values for more accuracy but slower performance -->
      <max_step_size>0.001</max_step_size>

      <!-- Real-time update rate - affects simulation smoothness -->
      <real_time_update_rate>1000</real_time_update_rate>

      <!-- Gravity - standard Earth gravity for humanoid testing -->
      <gravity>0 0 -9.8</gravity>

      <!-- ODE-specific parameters -->
      <ode>
        <solver>
          <!-- QuickStep solver for better stability with humanoid joints -->
          <type>quick</type>

          <!-- More iterations for better accuracy with complex humanoid models -->
          <iters>100</iters>

          <!-- Successive over-relaxation parameter for convergence -->
          <sor>1.3</sor>
        </solver>

        <constraints>
          <!-- Constraint Force Mixing - affects joint stability -->
          <cfm>1e-5</cfm>

          <!-- Error Reduction Parameter - affects constraint accuracy -->
          <erp>0.2</erp>

          <!-- Maximum velocity correction for contacts -->
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>

          <!-- Contact surface layer thickness -->
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Example humanoid robot with physics properties -->
    <model name="humanoid_robot">
      <pose>0 0 1 0 0 0</pose>

      <!-- Torso link with proper inertial properties -->
      <link name="torso">
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>0.3</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.4</iyy>
            <iyz>0.0</iyz>
            <izz>0.2</izz>
          </inertia>
        </inertial>

        <collision name="torso_collision">
          <geometry>
            <box>
              <size>0.3 0.2 0.5</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.5</mu>
                <mu2>0.5</mu2>
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

        <visual name="torso_visual">
          <geometry>
            <box>
              <size>0.3 0.2 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.2 1</ambient>
            <diffuse>0.9 0.7 0.3 1</diffuse>
          </material>
        </visual>
      </link>

      <!-- Left leg with proper joint constraints -->
      <link name="left_thigh">
        <pose>0 0.15 -0.25 0 0 0</pose>
        <inertial>
          <mass>3.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.1</iyy>
            <iyz>0.0</iyz>
            <izz>0.02</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Hip joint with appropriate limits -->
      <joint name="left_hip_joint" type="revolute">
        <parent>torso</parent>
        <child>left_thigh</child>
        <pose>0 0.15 -0.25 0 0 0</pose>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-1.57</lower>
            <upper>1.57</upper>
            <effort>100</effort>
            <velocity>2.0</velocity>
          </limit>
        </axis>
      </joint>

    </model>

  </world>
</sdf>
```

### Example 2: Advanced Collision Configuration
Configuring collision properties for realistic humanoid interactions:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="advanced_collision_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>200</iters>
          <sor>1.2</sor>
        </solver>
        <constraints>
          <cfm>1e-6</cfm>
          <erp>0.1</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Ground with high friction for humanoid stability -->
    <model name="high_friction_ground">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>2.0</mu>
                <mu2>2.0</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0.0</slip1>
                <slip2>0.0</slip2>
              </ode>
              <torsional>
                <coefficient>1.0</coefficient>
                <use_patch_radius>false</use_patch_radius>
                <surface_radius>0.01</surface_radius>
              </torsional>
            </friction>
            <contact>
              <ode>
                <kp>1e7</kp>
                <kd>1000</kd>
                <max_vel>100.0</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <bounce>
              <restitution_coefficient>0.01</restitution_coefficient>
              <threshold>100000</threshold>
            </bounce>
          </surface>
        </collision>
      </link>
    </model>

    <!-- Humanoid robot with detailed collision geometry -->
    <model name="detailed_humanoid">
      <pose>0 0 1 0 0 0</pose>

      <!-- Torso with capsule collision for better contact handling -->
      <link name="torso">
        <inertial>
          <mass>15.0</mass>
          <inertia>
            <ixx>0.5</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.6</iyy>
            <iyz>0.0</iyz>
            <izz>0.3</izz>
          </inertia>
        </inertial>

        <!-- Multiple collision elements for accurate contact detection -->
        <collision name="torso_collision_main">
          <geometry>
            <box>
              <size>0.3 0.25 0.6</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
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

        <!-- Additional collision for shoulders -->
        <collision name="shoulder_collision">
          <pose>0 0 0.2 0 0 0</pose>
          <geometry>
            <capsule>
              <radius>0.15</radius>
              <length>0.2</length>
            </capsule>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>0.6</mu2>
              </ode>
            </friction>
          </surface>
        </collision>

        <visual name="torso_visual">
          <geometry>
            <box>
              <size>0.3 0.25 0.6</size>
            </box>
          </geometry>
        </visual>
      </link>

      <!-- Feet with contact sensors and high friction -->
      <link name="left_foot">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.01</iyy>
            <iyz>0.0</iyz>
            <izz>0.005</izz>
          </inertia>
        </inertial>

        <collision name="left_foot_collision">
          <geometry>
            <box>
              <size>0.2 0.1 0.05</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.5</mu>
                <mu2>1.5</mu2>
                <!-- Anisotropic friction for forward/backward vs side motion -->
                <fdir1>1 0 0</fdir1>
              </ode>
            </friction>
            <contact>
              <ode>
                <kp>1e7</kp>
                <kd>1000</kd>
              </ode>
            </contact>
          </surface>
        </collision>

        <visual name="left_foot_visual">
          <geometry>
            <box>
              <size>0.2 0.1 0.05</size>
            </box>
          </geometry>
        </visual>
      </link>

    </model>

  </world>
</sdf>
```

### Example 3: Python Physics Parameter Tuning
Python script for tuning physics parameters during simulation:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from gazebo_msgs.msg import ODEPhysics
from std_srvs.srv import Empty
import time

class PhysicsTuner(Node):
    def __init__(self):
        super().__init__('physics_tuner')

        # Create clients for physics services
        self.get_physics_client = self.create_client(
            GetPhysicsProperties,
            '/get_physics_properties'
        )
        self.set_physics_client = self.create_client(
            SetPhysicsProperties,
            '/set_physics_properties'
        )
        self.pause_physics_client = self.create_client(
            Empty,
            '/pause_physics'
        )
        self.unpause_physics_client = self.create_client(
            Empty,
            '/unpause_physics'
        )

        # Wait for services to be available
        while not self.get_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_physics_properties service...')

        while not self.set_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_physics_properties service...')

        self.get_logger().info('Physics tuner initialized')

    def get_current_physics_properties(self):
        """Get current physics properties from Gazebo"""
        request = GetPhysicsProperties.Request()
        future = self.get_physics_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Current physics properties:')
            self.get_logger().info(f'  Time step: {response.time_step}')
            self.get_logger().info(f'  Max update rate: {response.max_update_rate}')
            self.get_logger().info(f'  Gravity: {response.gravity}')
            return response
        else:
            self.get_logger().error('Failed to get physics properties')
            return None

    def set_physics_properties(self, time_step, max_update_rate, gravity, ode_config):
        """Set new physics properties"""
        request = SetPhysicsProperties.Request()

        # Set basic properties
        request.time_step = time_step
        request.max_update_rate = max_update_rate

        # Set gravity
        request.gravity = gravity

        # Set ODE physics parameters
        request.ode_config = ode_config

        future = self.set_physics_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Physics properties updated successfully')
            return True
        else:
            self.get_logger().error('Failed to set physics properties')
            return False

    def tune_for_humanoid_simulation(self):
        """Tune physics for humanoid robot simulation"""
        self.get_logger().info('Tuning physics for humanoid simulation...')

        # First, pause physics to avoid instability during changes
        self.pause_physics()

        # Get current properties as base
        current_props = self.get_current_physics_properties()
        if not current_props:
            return False

        # Create new ODE configuration optimized for humanoid robots
        ode_config = ODEPhysics()
        ode_config.auto_disable_bodies = False
        ode_config.sor_pgs_precon_iters = 0
        ode_config.sor_pgs_iters = 100  # More iterations for stability
        ode_config.sor_pgs_w = 1.3      # SOR parameter for convergence
        ode_config.ode_slip = 0.0
        ode_config.ode_friction_model = "pyramid_model"  # More accurate friction

        # Set new physics properties
        success = self.set_physics_properties(
            time_step=0.001,  # 1ms time step for accuracy
            max_update_rate=1000.0,  # 1000 Hz update rate
            gravity=current_props.gravity,  # Keep same gravity
            ode_config=ode_config
        )

        # Resume physics
        self.unpause_physics()

        if success:
            self.get_logger().info('Physics tuned successfully for humanoid simulation')
            return True
        else:
            self.get_logger().error('Failed to tune physics for humanoid simulation')
            return False

    def pause_physics(self):
        """Pause physics simulation"""
        request = Empty.Request()
        future = self.pause_physics_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Physics paused')

    def unpause_physics(self):
        """Resume physics simulation"""
        request = Empty.Request()
        future = self.unpause_physics_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Physics resumed')

    def test_collision_detection(self):
        """Test collision detection and response"""
        self.get_logger().info('Testing collision detection...')

        # This would typically involve spawning objects and monitoring contacts
        # For this example, we'll just log the process
        self.get_logger().info('Collision detection test would involve:')
        self.get_logger().info('1. Spawning test objects')
        self.get_logger().info('2. Monitoring contact sensors')
        self.get_logger().info('3. Verifying collision responses')
        self.get_logger().info('4. Adjusting parameters based on results')

def main(args=None):
    rclpy.init(args=args)

    physics_tuner = PhysicsTuner()

    # Tune physics for humanoid simulation
    success = physics_tuner.tune_for_humanoid_simulation()

    if success:
        physics_tuner.get_logger().info('Physics tuning completed successfully')

        # Test collision detection
        physics_tuner.test_collision_detection()
    else:
        physics_tuner.get_logger().error('Physics tuning failed')

    # Keep the node running
    try:
        rclpy.spin(physics_tuner)
    except KeyboardInterrupt:
        physics_tuner.get_logger().info('Shutting down physics tuner')
    finally:
        physics_tuner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Simulation Exercises

### Exercise 1: Physics Parameter Optimization
- **Objective**: Optimize physics parameters for humanoid robot simulation
- **Requirements**: Gazebo, ROS 2, humanoid robot model
- **Steps**:
  1. Start with default physics parameters
  2. Test robot stability and joint behavior
  3. Gradually adjust time step, solver iterations, and constraints
  4. Measure simulation accuracy vs performance trade-offs
- **Expected Outcome**: Optimized physics parameters for stable humanoid simulation

### Exercise 2: Gravity Environment Testing
- **Objective**: Test robot behavior under different gravity conditions
- **Requirements**: Gazebo, robot model, physics configuration
- **Steps**:
  1. Configure gravity for Earth (9.8 m/s²)
  2. Test robot walking and balance
  3. Adjust to Moon gravity (1.6 m/s²)
  4. Observe and document behavioral differences
- **Expected Outcome**: Understanding of gravity effects on humanoid locomotion

### Exercise 3: Collision Response Tuning
- **Objective**: Tune collision parameters for realistic interactions
- **Requirements**: Gazebo, robot model, various objects
- **Steps**:
  1. Configure collision properties for robot feet
  2. Test walking on different surface types
  3. Adjust friction and contact parameters
  4. Validate realistic contact behavior
- **Expected Outcome**: Properly tuned collision responses for humanoid robot

## Summary

Physics simulation in Gazebo involves configuring multiple parameters including time steps, solver settings, gravity, and collision properties to achieve realistic behavior for humanoid robots. Proper tuning of these parameters is essential for stable simulation and accurate representation of real-world physics. The balance between simulation accuracy and computational performance requires careful consideration based on the specific requirements of the humanoid robot being simulated.

## Key Terms

- **Time Step**: Duration of each physics simulation step, smaller values increase accuracy but decrease performance
- **Solver Iterations**: Number of iterations the physics solver performs to resolve constraints
- **Constraint Force Mixing (CFM)**: Parameter that affects constraint stability
- **Error Reduction Parameter (ERP)**: Parameter that affects how quickly constraint errors are corrected
- **Contact Stiffness/Damping**: Parameters that control collision response behavior
- **Friction Coefficients**: Values that determine sliding resistance between surfaces
- **Inertial Properties**: Mass, center of mass, and moments of inertia for objects
- **Collision Mesh**: Geometry used for collision detection (different from visual mesh)

## Next Steps

Continue to Lesson 5.3: "Sensor Simulation: LiDAR, IMU, Depth Cameras" to explore how to simulate various sensors in Gazebo for humanoid robots.
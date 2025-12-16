---
sidebar_label: 'Lesson 7.2: Isaac Sim Python API'
---

# Lesson 7.2: Isaac Sim Python API

## Overview

Isaac Sim provides a comprehensive Python API that enables programmatic control of simulations, robot creation, sensor configuration, and integration with external systems. This lesson explores the core components of the Isaac Sim Python API and demonstrates practical applications for robotics development.

## Learning Objectives

By the end of this lesson, you will be able to:
- Utilize the Isaac Sim core classes and modules
- Programmatically create and configure robots
- Configure and manage sensors in simulations
- Control simulation flow and timing
- Integrate external systems with Isaac Sim

## Core Isaac Sim Classes

### World Class

The `World` class is the primary interface for managing simulations:

```python
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# Create a world instance
world = World(stage_units_in_meters=1.0)

# Reset the world to initial state
world.reset()

# Step the simulation
world.step(render=True)

# Get current simulation time
current_time = world.current_time
print(f"Simulation time: {current_time}")

# Access the world interface
world_interface = world.scene
```

### Robot Creation and Management

```python
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

class MyRobot(Robot):
    def __init__(
        self,
        prim_path: str,
        name: str = "my_robot",
        usd_path: str = None,
        position: np.ndarray = np.array([0, 0, 0]),
        orientation: np.ndarray = np.array([0, 0, 0, 1]),
    ) -> None:
        self._usd_path = usd_path
        self._position = position
        self._orientation = orientation
        super().__init__(
            prim_path=prim_path,
            name=name,
            usd_path=usd_path,
            position=position,
            orientation=orientation,
        )

    def initialize(self, world):
        super().initialize(world=world)
        # Additional initialization code for custom robot
        self._default_joints_state = self.get_joints_state()
        self._articulation_controller = self.get_articulation_controller()

    def get_joints_state(self):
        positions = self.get_joint_positions()
        velocities = self.get_joint_velocities()
        return {"positions": positions, "velocities": velocities}

    def apply_actions(self, actions):
        # Apply joint position or velocity commands
        self._articulation_controller.apply_pos_cmd(actions)

# Example usage
my_robot = MyRobot(
    prim_path="/World/MyRobot",
    name="my_custom_robot",
    usd_path="path/to/robot.usd",  # Replace with actual path
    position=np.array([0, 0, 0.5])
)
```

### Scene Management

```python
from omni.isaac.core.scenes import Scene
from omni.isaac.core.prims import RigidPrim, XFormPrim
from omni.isaac.core.utils.prims import create_prim
import omni.isaac.core.utils.prims as prim_utils

# Create a custom scene
class MyRoboticsScene(Scene):
    def __init__(self, scene_path="/World", name="robotics_scene"):
        super().__init__(scene_path=scene_path, name=name)

    def add_ground_plane(self, size=10.0):
        # Add a ground plane to the scene
        prim_utils.create_prim(
            prim_path="/World/GroundPlane",
            prim_type="Plane",
            position=np.array([0, 0, 0]),
            orientation=np.array([0, 0, 0, 1]),
            scale=np.array([size, size, 1])
        )

    def add_obstacles(self):
        # Add some simple obstacles
        obstacle_positions = [
            [2, 0, 0.5],
            [0, 2, 0.5],
            [-2, 0, 0.5],
            [0, -2, 0.5]
        ]

        for i, pos in enumerate(obstacle_positions):
            prim_utils.create_prim(
                prim_path=f"/World/Obstacle_{i}",
                prim_type="Cube",
                position=np.array(pos),
                scale=np.array([0.5, 0.5, 1.0])
            )

# Example usage
scene = MyRoboticsScene()
scene.add_ground_plane()
scene.add_obstacles()
```

## Sensor Integration

### Camera Sensors

```python
from omni.isaac.sensor import Camera
import carb
import numpy as np

class CameraRobot(Robot):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._camera = None

    def add_camera(self, prim_path, position, orientation):
        # Create a camera sensor
        self._camera = Camera(
            prim_path=prim_path,
            frequency=30,  # Hz
            resolution=(640, 480)
        )
        self._camera.set_focal_length(24.0)  # mm
        self._camera.set_horizontal_aperture(20.955)  # mm
        self._camera.set_vertical_aperture(15.29)  # mm

        # Position the camera relative to the robot
        self._camera.set_translation(position)
        self._camera.set_rotation(orientation)

        return self._camera

    def get_camera_data(self):
        # Get RGB image data
        rgb_data = self._camera.get_rgb()

        # Get depth data
        depth_data = self._camera.get_depth()

        # Get point cloud
        point_cloud = self._camera.get_point_cloud()

        return {
            "rgb": rgb_data,
            "depth": depth_data,
            "point_cloud": point_cloud
        }

# Example usage
camera_robot = CameraRobot(
    prim_path="/World/CameraRobot",
    name="camera_robot"
)

# Add a camera to the robot
camera = camera_robot.add_camera(
    prim_path="/World/CameraRobot/Camera",
    position=np.array([0.2, 0, 0.1]),
    orientation=np.array([0, 0, 0, 1])
)
```

### LiDAR Sensors

```python
from omni.isaac.range_sensor import _range_sensor
import numpy as np

class LidarRobot(Robot):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
        self._lidar_sensor = None

    def add_lidar(self, prim_path, position, orientation):
        # Create a LiDAR sensor
        from omni.isaac.core.utils.prims import create_prim

        # Create the LiDAR sensor prim
        create_prim(
            prim_path=prim_path,
            prim_type="Xform",
            position=position,
            orientation=orientation
        )

        # Configure LiDAR parameters
        self._lidar_sensor = self._lidar_interface.new_lidar(
            prim_path,
            translation=position,
            orientation=orientation,
            min_range=0.1,
            max_range=25.0,
            draw_points=True,
            draw_lines=True,
            horizontal_samples=1080,
            vertical_samples=64,
            horizontal_fov=360.0,
            vertical_fov=30.0
        )

        return self._lidar_sensor

    def get_lidar_data(self):
        # Get LiDAR scan data
        scan_data = self._lidar_interface.get_linear_depth_data(
            self._lidar_sensor.GetPath().pathString
        )
        return scan_data

# Example usage
lidar_robot = LidarRobot(
    prim_path="/World/LidarRobot",
    name="lidar_robot"
)

# Add a LiDAR sensor to the robot
lidar = lidar_robot.add_lidar(
    prim_path="/World/LidarRobot/Lidar",
    position=np.array([0.1, 0, 0.3]),
    orientation=np.array([0, 0, 0, 1])
)
```

### IMU Sensors

```python
from omni.isaac.core.sensors import ImuSensor
from omni.isaac.core.utils.prims import get_prim_at_path

class IMURobot(Robot):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._imu_sensor = None

    def add_imu(self, prim_path, position, orientation):
        # Create an IMU sensor
        self._imu_sensor = ImuSensor(
            prim_path=prim_path,
            name="imu_sensor",
            position=position,
            orientation=orientation
        )

        return self._imu_sensor

    def get_imu_data(self):
        # Get IMU data
        linear_acceleration = self._imu_sensor.get_linear_acceleration()
        angular_velocity = self._imu_sensor.get_angular_velocity()
        orientation = self._imu_sensor.get_orientation()

        return {
            "linear_acceleration": linear_acceleration,
            "angular_velocity": angular_velocity,
            "orientation": orientation
        }

# Example usage
imu_robot = IMURobot(
    prim_path="/World/IMURobot",
    name="imu_robot"
)

# Add an IMU to the robot
imu = imu_robot.add_imu(
    prim_path="/World/IMURobot/IMU",
    position=np.array([0, 0, 0.2]),
    orientation=np.array([0, 0, 0, 1])
)
```

## Simulation Control

### Custom Simulation Loop

```python
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core import World
import numpy as np
import asyncio

class CustomSimulationLoop:
    def __init__(self, world):
        self.world = world
        self.simulation_time = 0.0
        self.target_framerate = 60.0
        self.dt = 1.0 / self.target_framerate

    async def run_simulation(self, duration=10.0):
        """Run simulation for a specific duration"""
        end_time = self.simulation_time + duration

        while self.simulation_time < end_time:
            # Pre-step operations
            self.pre_step_operations()

            # Step the world
            self.world.step(render=True)

            # Post-step operations
            self.post_step_operations()

            # Update simulation time
            self.simulation_time += self.dt

            # Optional: add delay for real-time simulation
            await asyncio.sleep(0)  # Yield control to other coroutines

    def pre_step_operations(self):
        """Operations to perform before each simulation step"""
        # Update robot controllers
        # Process sensor data
        # Apply control commands
        pass

    def post_step_operations(self):
        """Operations to perform after each simulation step"""
        # Collect sensor data
        # Log simulation state
        # Check termination conditions
        pass

# Example usage
world = World(stage_units_in_meters=1.0)
sim_loop = CustomSimulationLoop(world)

# Run the simulation loop
# asyncio.run(sim_loop.run_simulation(duration=10.0))
```

### Real-time Control Interface

```python
import threading
import time
from queue import Queue

class RealTimeController:
    def __init__(self, robot, world):
        self.robot = robot
        self.world = world
        self.command_queue = Queue()
        self.current_state = None
        self.is_running = False
        self.control_thread = None

    def start_control_loop(self):
        """Start the real-time control loop in a separate thread"""
        self.is_running = True
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.start()

    def stop_control_loop(self):
        """Stop the real-time control loop"""
        self.is_running = False
        if self.control_thread:
            self.control_thread.join()

    def send_command(self, command):
        """Send a command to the robot"""
        self.command_queue.put(command)

    def _control_loop(self):
        """Internal control loop running in separate thread"""
        while self.is_running:
            # Get current robot state
            self.current_state = self.robot.get_joints_state()

            # Process commands from queue
            while not self.command_queue.empty():
                command = self.command_queue.get()
                self._execute_command(command)

            # Small delay to prevent busy waiting
            time.sleep(0.001)  # 1ms delay

    def _execute_command(self, command):
        """Execute a specific command"""
        if command["type"] == "joint_position":
            self.robot.apply_actions(command["data"])
        elif command["type"] == "velocity":
            self.robot._articulation_controller.apply_vel_cmd(command["data"])
        elif command["type"] == "effort":
            self.robot._articulation_controller.apply_effort_cmd(command["data"])
```

## ROS Integration

### ROS Bridge Setup

```python
from omni.isaac.core.utils.extensions import enable_extension

# Enable ROS bridge extension
enable_extension("omni.isaac.ros_bridge")

# ROS publisher example
def setup_ros_publisher(robot):
    """Setup ROS publishers for robot data"""
    import rospy
    from std_msgs.msg import Float32MultiArray
    from sensor_msgs.msg import JointState

    # Initialize ROS node
    rospy.init_node("isaac_sim_robot_controller", anonymous=True)

    # Create publishers
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    robot_state_pub = rospy.Publisher("/robot_state", Float32MultiArray, queue_size=10)

    return joint_pub, robot_state_pub

# ROS subscriber example
def setup_ros_subscriber(robot):
    """Setup ROS subscribers for commands"""
    import rospy
    from std_msgs.msg import Float32MultiArray
    from geometry_msgs.msg import Twist

    def cmd_vel_callback(msg):
        # Convert Twist message to robot commands
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Apply to robot (implementation specific)
        # robot.apply_differential_drive(linear_vel, angular_vel)

    # Subscribe to command velocity topic
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
```

## Practical Exercise

Create a complete Isaac Sim application with:

1. A differential drive robot with two wheels
2. RGB camera and LiDAR sensors
3. Real-time control interface
4. Data logging and visualization
5. Basic navigation simulation

## Summary

The Isaac Sim Python API provides powerful tools for creating sophisticated robotics simulations. Its object-oriented design and comprehensive functionality enable the development of complex robotic systems with realistic sensors, accurate physics, and seamless integration with external frameworks.

## Next Steps

The next lesson will focus on advanced Isaac Sim features, including domain randomization, reinforcement learning integration, and large-scale environment creation for AI training.
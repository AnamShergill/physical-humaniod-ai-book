---
sidebar_label: 'Lesson 7.3: Advanced Isaac Sim Features'
---

# Lesson 7.3: Advanced Isaac Sim Features

## Overview

This lesson explores advanced features of Isaac Sim that enable sophisticated robotics applications, including domain randomization for robust AI training, reinforcement learning integration, large-scale environment generation, and advanced sensor simulation techniques. These features position Isaac Sim as a premier platform for developing AI-powered robotic systems.

## Learning Objectives

By the end of this lesson, you will be able to:
- Implement domain randomization for robust AI training
- Integrate reinforcement learning frameworks with Isaac Sim
- Generate large-scale diverse environments
- Utilize advanced sensor simulation techniques
- Optimize simulations for AI training workflows

## Domain Randomization

### Introduction to Domain Randomization

Domain randomization is a technique that randomizes various aspects of the simulation environment to make AI models more robust when deployed to the real world. By training on diverse simulated environments, models become less sensitive to specific environmental conditions.

### Material Randomization

```python
from omni.isaac.core.materials import PreviewSurface
from omni.isaac.core.utils.prims import create_prim
from pxr import Gf, UsdShade
import random
import numpy as np

class MaterialRandomizer:
    def __init__(self):
        self.material_properties = {
            "roughness_range": (0.1, 0.9),
            "metallic_range": (0.0, 0.2),
            "specular_range": (0.1, 1.0),
            "diffuse_color_range": (
                (0.2, 0.2, 0.2),  # Min RGB
                (0.8, 0.8, 0.8)   # Max RGB
            )
        }

    def create_random_material(self, prim_path):
        """Create a material with randomized properties"""
        # Create a preview surface material
        material = PreviewSurface(
            prim_path=prim_path,
            color=(random.uniform(0.2, 0.8), random.uniform(0.2, 0.8), random.uniform(0.2, 0.8)),
            roughness=random.uniform(0.1, 0.9),
            metallic=random.uniform(0.0, 0.2),
            specular=random.uniform(0.1, 1.0)
        )
        return material

    def apply_random_material_to_prim(self, prim_path):
        """Apply a random material to an existing prim"""
        material_path = f"{prim_path}_Material"
        material = self.create_random_material(material_path)

        # Bind the material to the prim
        material.bind(prim_path)
        return material

    def randomize_existing_material(self, material_prim_path):
        """Randomize properties of an existing material"""
        stage = omni.usd.get_context().get_stage()
        material_prim = stage.GetPrimAtPath(material_prim_path)

        if material_prim and material_prim.IsValid():
            # Get the shader
            shader_path = material_prim.GetPath().AppendChild("OmniPBR")
            shader = UsdShade.Shader(stage.GetPrimAtPath(shader_path))

            if shader:
                # Randomize shader parameters
                shader.GetInput("roughness").Set(random.uniform(0.1, 0.9))
                shader.GetInput("metallic").Set(random.uniform(0.0, 0.2))
                shader.GetInput("diffuse_color").Set(
                    Gf.Vec3f(
                        random.uniform(0.2, 0.8),
                        random.uniform(0.2, 0.8),
                        random.uniform(0.2, 0.8)
                    )
                )

# Example usage
material_randomizer = MaterialRandomizer()

# Apply random materials to objects in the scene
for i in range(10):
    object_path = f"/World/Object_{i}"
    material_randomizer.apply_random_material_to_prim(object_path)
```

### Lighting Randomization

```python
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import get_stage_units
import random

class LightingRandomizer:
    def __init__(self):
        self.light_properties = {
            "intensity_range": (100, 10000),
            "color_temperature_range": (3000, 8000),
            "position_range": ((-5, -5, 5), (5, 5, 10)),
            "direction_range": ((-1, -1, -1), (1, 1, 1))
        }

    def create_random_distant_light(self, light_name="RandomDistantLight"):
        """Create a distant light with randomized properties"""
        light_path = f"/World/{light_name}"

        # Create distant light
        create_prim(
            prim_path=light_path,
            prim_type="DistantLight",
            position=[
                random.uniform(-5, 5),
                random.uniform(-5, 5),
                random.uniform(5, 10)
            ]
        )

        # Get the light prim and set properties
        stage = omni.usd.get_context().get_stage()
        light_prim = stage.GetPrimAtPath(light_path)

        # Set randomized properties
        light_prim.GetAttribute("inputs:intensity").Set(
            random.uniform(100, 10000)
        )

        light_prim.GetAttribute("inputs:color").Set(
            self._kelvin_to_rgb(random.uniform(3000, 8000))
        )

        return light_path

    def create_random_dome_light(self, light_name="RandomDomeLight"):
        """Create a dome light with randomized HDRI environment"""
        light_path = f"/World/{light_name}"

        # Create dome light
        create_prim(
            prim_path=light_path,
            prim_type="DomeLight"
        )

        # Set randomized properties
        stage = omni.usd.get_context().get_stage()
        light_prim = stage.GetPrimAtPath(light_path)

        # Randomize intensity and color
        light_prim.GetAttribute("inputs:color").Set(
            (random.uniform(0.5, 1.0), random.uniform(0.5, 1.0), random.uniform(0.5, 1.0))
        )

        light_prim.GetAttribute("inputs:intensity").Set(
            random.uniform(1000, 5000)
        )

        return light_path

    def _kelvin_to_rgb(self, kelvin):
        """Convert color temperature in Kelvin to RGB"""
        temp = kelvin / 100
        red, green, blue = 0, 0, 0

        # Red calculation
        if temp <= 66:
            red = 1.0
        else:
            red = temp - 60
            red = 329.698727446 * (red ** -0.1332047592)
            red = max(0, min(1, red / 255))

        # Green calculation
        if temp <= 66:
            green = temp
            green = 99.4708025861 * np.log(green) - 161.1195681661
        else:
            green = temp - 60
            green = 288.1221695283 * (green ** -0.0755148492)
        green = max(0, min(1, green / 255))

        # Blue calculation
        if temp >= 66:
            blue = 1.0
        elif temp <= 19:
            blue = 0.0
        else:
            blue = temp - 10
            blue = 138.5177312231 * np.log(blue) - 305.0447927307
            blue = max(0, min(1, blue / 255))

        return (red, green, blue)

    def randomize_lighting_environment(self):
        """Randomize the entire lighting setup"""
        # Remove existing lights
        self._remove_existing_lights()

        # Add random lights
        light_types = ["distant", "dome", "sphere"]
        num_lights = random.randint(1, 3)

        for i in range(num_lights):
            light_type = random.choice(light_types)
            if light_type == "distant":
                self.create_random_distant_light(f"RandomDistantLight_{i}")
            elif light_type == "dome":
                self.create_random_dome_light(f"RandomDomeLight_{i}")
            elif light_type == "sphere":
                self.create_random_sphere_light(f"RandomSphereLight_{i}")

    def _remove_existing_lights(self):
        """Remove all existing lights from the scene"""
        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if prim.GetTypeName() in ["DistantLight", "DomeLight", "SphereLight", "RectLight"]:
                stage.RemovePrim(prim.GetPath())

    def create_random_sphere_light(self, light_name="RandomSphereLight"):
        """Create a sphere light with randomized properties"""
        light_path = f"/World/{light_name}"

        # Create sphere light
        create_prim(
            prim_path=light_path,
            prim_type="SphereLight",
            position=[
                random.uniform(-3, 3),
                random.uniform(-3, 3),
                random.uniform(2, 6)
            ]
        )

        # Set randomized properties
        stage = omni.usd.get_context().get_stage()
        light_prim = stage.GetPrimAtPath(light_path)

        light_prim.GetAttribute("inputs:intensity").Set(
            random.uniform(500, 5000)
        )

        light_prim.GetAttribute("inputs:color").Set(
            (random.uniform(0.5, 1.0), random.uniform(0.5, 1.0), random.uniform(0.5, 1.0))
        )

        return light_path

# Example usage
lighting_randomizer = LightingRandomizer()
lighting_randomizer.randomize_lighting_environment()
```

### Physics Parameter Randomization

```python
import random
from omni.isaac.core.utils.prims import get_prim_at_path

class PhysicsRandomizer:
    def __init__(self):
        self.physics_properties = {
            "friction_range": (0.1, 1.0),
            "restitution_range": (0.0, 0.5),
            "density_range": (100, 2000),
            "linear_damping_range": (0.0, 10.0),
            "angular_damping_range": (0.0, 1.0)
        }

    def randomize_rigid_body_properties(self, prim_path):
        """Randomize physical properties of a rigid body"""
        prim = get_prim_at_path(prim_path)

        if prim:
            # Randomize friction
            friction = random.uniform(0.1, 1.0)
            if prim.HasAttribute("physics:friction"):
                prim.GetAttribute("physics:friction").Set(friction)
            else:
                prim.CreateAttribute("physics:friction", Sdf.ValueTypeNames.Float).Set(friction)

            # Randomize restitution (bounciness)
            restitution = random.uniform(0.0, 0.5)
            if prim.HasAttribute("physics:restitution"):
                prim.GetAttribute("physics:restitution").Set(restitution)
            else:
                prim.CreateAttribute("physics:restitution", Sdf.ValueTypeNames.Float).Set(restitution)

            # Randomize mass properties
            density = random.uniform(100, 2000)
            if prim.HasAttribute("physics:density"):
                prim.GetAttribute("physics:density").Set(density)
            else:
                prim.CreateAttribute("physics:density", Sdf.ValueTypeNames.Float).Set(density)

    def randomize_joint_properties(self, joint_prim_path):
        """Randomize properties of a joint"""
        joint_prim = get_prim_at_path(joint_prim_path)

        if joint_prim:
            # Randomize joint limits
            joint_prim.GetAttribute("physics:jointLinearLimit").Set(
                random.uniform(0.1, 1.0)
            )

            # Randomize joint drive properties
            joint_prim.GetAttribute("physics:driveStrength").Set(
                random.uniform(100, 1000)
            )

    def randomize_simulation_parameters(self):
        """Randomize global simulation parameters"""
        # Randomize gravity
        gravity = [
            random.uniform(-0.5, 0.5),
            random.uniform(-0.5, 0.5),
            random.uniform(-10.5, -9.5)  # Keep Z (down) dominant
        ]

        from omni.isaac.core import World
        world = World()
        world.scene.enable_gravity = True
        # Note: In practice, you would set gravity through physics scene setup

# Example usage
physics_randomizer = PhysicsRandomizer()

# Randomize properties for all objects in the scene
stage = omni.usd.get_context().get_stage()
for prim in stage.Traverse():
    if prim.GetTypeName() in ["Cylinder", "Cube", "Sphere", "Mesh"]:
        physics_randomizer.randomize_rigid_body_properties(prim.GetPath().pathString)
```

## Reinforcement Learning Integration

### RL Environment Setup

```python
import gym
from gym import spaces
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import Articulation

class IsaacSimRLEnvironment(gym.Env):
    """Gym environment wrapper for Isaac Sim"""

    def __init__(self, world, robot, target_position):
        super(IsaacSimRLEnvironment, self).__init__()

        self.world = world
        self.robot = robot
        self.target_position = target_position

        # Define action and observation spaces
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32  # Differential drive: [linear, angular]
        )

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(10,), dtype=np.float32  # Example observation
        )

        self.max_steps = 1000
        self.current_step = 0
        self.robot_start_position = [0, 0, 0.1]

    def reset(self):
        """Reset the environment to initial state"""
        # Reset simulation
        self.world.reset()
        self.current_step = 0

        # Reset robot position
        self.robot.set_world_poses(
            positions=np.array([self.robot_start_position]),
            orientations=np.array([[0, 0, 0, 1]])
        )

        # Randomize target position slightly
        self.target_position = [
            np.random.uniform(-3, 3),
            np.random.uniform(-3, 3),
            0.1
        ]

        return self._get_observation()

    def step(self, action):
        """Execute one step in the environment"""
        # Apply action to robot
        self._apply_action(action)

        # Step the simulation
        self.world.step(render=True)

        # Get observation
        observation = self._get_observation()

        # Calculate reward
        reward = self._calculate_reward()

        # Check termination conditions
        done = self._check_termination()

        # Update step count
        self.current_step += 1

        # Additional info
        info = {
            "step": self.current_step,
            "distance_to_target": self._get_distance_to_target()
        }

        return observation, reward, done, info

    def _apply_action(self, action):
        """Apply action to the robot"""
        # For differential drive robot
        linear_vel = action[0] * 1.0  # Scale factor
        angular_vel = action[1] * 1.0  # Scale factor

        # Apply velocities to robot (implementation specific)
        # This would involve setting joint velocities or using a controller
        pass

    def _get_observation(self):
        """Get current observation from the environment"""
        # Get robot state (position, velocity, etc.)
        robot_position, robot_orientation = self.robot.get_world_poses()
        robot_linear_vel, robot_angular_vel = self.robot.get_velocities()

        # Get target position
        target_pos = np.array(self.target_position)

        # Calculate relative position
        relative_pos = target_pos - robot_position[0]

        # Combine all observations
        observation = np.concatenate([
            robot_position[0],           # Robot position
            robot_linear_vel[:2],        # Robot velocity (x, y)
            relative_pos[:2],            # Relative target position
            [self.current_step / self.max_steps]  # Normalized step count
        ])

        return observation.astype(np.float32)

    def _calculate_reward(self):
        """Calculate reward based on current state"""
        # Distance-based reward
        distance = self._get_distance_to_target()
        distance_reward = -distance  # Negative distance encourages getting closer

        # Success reward
        if distance < 0.5:  # Within 0.5m of target
            return 100.0

        # Small time penalty to encourage efficiency
        time_penalty = -0.01

        return distance_reward + time_penalty

    def _get_distance_to_target(self):
        """Calculate distance from robot to target"""
        robot_pos, _ = self.robot.get_world_poses()
        target_pos = np.array(self.target_position)

        distance = np.linalg.norm(robot_pos[0][:2] - target_pos[:2])
        return distance

    def _check_termination(self):
        """Check if episode should terminate"""
        # Check if max steps reached
        if self.current_step >= self.max_steps:
            return True

        # Check if reached target
        if self._get_distance_to_target() < 0.5:
            return True

        # Check for collisions or other failure conditions
        # (implementation specific)

        return False
```

### Training Loop Integration

```python
import torch
import torch.nn as nn
import torch.optim as optim
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.env_util import make_vec_env

class RLTrainingManager:
    def __init__(self, env_class, world, robot, target_position):
        self.env_class = env_class
        self.world = world
        self.robot = robot
        self.target_position = target_position
        self.model = None

    def create_environment(self):
        """Create the RL environment"""
        env = self.env_class(self.world, self.robot, self.target_position)
        return env

    def train_model(self, algorithm="PPO", total_timesteps=100000):
        """Train an RL model"""
        # Create vectorized environment
        env = make_vec_env(
            lambda: self.env_class(self.world, self.robot, self.target_position),
            n_envs=4  # Number of parallel environments
        )

        # Choose algorithm
        if algorithm == "PPO":
            self.model = PPO(
                "MlpPolicy",
                env,
                verbose=1,
                tensorboard_log="./tensorboard_logs/",
                learning_rate=3e-4,
                n_steps=2048,
                batch_size=64,
                n_epochs=10,
                gamma=0.99,
                gae_lambda=0.95,
                clip_range=0.2
            )
        elif algorithm == "SAC":
            self.model = SAC(
                "MlpPolicy",
                env,
                verbose=1,
                tensorboard_log="./tensorboard_logs/",
                learning_rate=3e-4,
                buffer_size=100000,
                learning_starts=1000,
                batch_size=256,
                tau=0.005,
                gamma=0.99
            )

        # Train the model
        self.model.learn(total_timesteps=total_timesteps)

        # Save the model
        self.model.save("isaac_sim_rl_model")

    def evaluate_model(self, num_episodes=10):
        """Evaluate the trained model"""
        if self.model is None:
            print("No model to evaluate. Train a model first.")
            return

        env = self.env_class(self.world, self.robot, self.target_position)
        successes = 0

        for episode in range(num_episodes):
            obs = env.reset()
            done = False
            total_reward = 0

            while not done:
                action, _states = self.model.predict(obs, deterministic=True)
                obs, reward, done, info = env.step(action)
                total_reward += reward

                if done:
                    if info.get("distance_to_target", float('inf')) < 0.5:
                        successes += 1
                    break

        success_rate = successes / num_episodes
        print(f"Success rate: {success_rate * 100:.2f}%")

# Example usage (requires stable-baselines3 installation)
# rl_manager = RLTrainingManager(IsaacSimRLEnvironment, world, robot, [2, 2, 0.1])
# rl_manager.train_model(algorithm="PPO", total_timesteps=50000)
# rl_manager.evaluate_model(num_episodes=20)
```

## Large-Scale Environment Generation

### Procedural Environment Generator

```python
import random
import numpy as np
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.nucleus import get_assets_root_path

class ProceduralEnvironmentGenerator:
    def __init__(self):
        self.environment_config = {
            "room_size_range": (5, 15),
            "obstacle_count_range": (3, 10),
            "obstacle_size_range": (0.2, 1.0),
            "object_categories": ["furniture", "decorations", "appliances"]
        }

    def generate_random_room(self, room_name="RandomRoom", position=[0, 0, 0]):
        """Generate a random room with walls, floor, and obstacles"""
        room_size = random.uniform(5, 15)

        # Create room structure
        room_path = f"/World/{room_name}"

        # Create floor
        floor_path = f"{room_path}/Floor"
        create_prim(
            prim_path=floor_path,
            prim_type="Plane",
            position=[position[0], position[1], position[2]],
            scale=[room_size, room_size, 1]
        )

        # Create walls
        wall_height = 3.0
        wall_thickness = 0.1

        wall_positions = [
            [position[0], position[1] + room_size/2, wall_height/2],  # North wall
            [position[0], position[1] - room_size/2, wall_height/2],  # South wall
            [position[0] + room_size/2, position[1], wall_height/2],  # East wall
            [position[0] - room_size/2, position[1], wall_height/2]   # West wall
        ]

        wall_sizes = [
            [room_size, wall_thickness, wall_height],  # North
            [room_size, wall_thickness, wall_height],  # South
            [wall_thickness, room_size, wall_height],  # East
            [wall_thickness, room_size, wall_height]   # West
        ]

        for i, (pos, size) in enumerate(zip(wall_positions, wall_sizes)):
            wall_path = f"{room_path}/Wall_{i}"
            create_prim(
                prim_path=wall_path,
                prim_type="Cube",
                position=pos,
                scale=size
            )

        # Add random obstacles
        self._add_random_obstacles(room_path, room_size, position)

        return room_path

    def _add_random_obstacles(self, room_path, room_size, room_center):
        """Add random obstacles to the room"""
        num_obstacles = random.randint(3, 10)

        for i in range(num_obstacles):
            # Random position within room bounds (with margin)
            margin = 1.0
            pos_x = random.uniform(
                room_center[0] - room_size/2 + margin,
                room_center[0] + room_size/2 - margin
            )
            pos_y = random.uniform(
                room_center[1] - room_size/2 + margin,
                room_center[1] + room_size/2 - margin
            )

            obstacle_size = random.uniform(0.2, 1.0)

            obstacle_path = f"{room_path}/Obstacle_{i}"
            obstacle_type = random.choice(["Cube", "Cylinder", "Sphere"])

            create_prim(
                prim_path=obstacle_path,
                prim_type=obstacle_type,
                position=[pos_x, pos_y, obstacle_size/2],  # Place on ground
                scale=[obstacle_size] * 3
            )

    def generate_warehouse_environment(self, warehouse_name="Warehouse", size=(20, 20)):
        """Generate a warehouse-style environment with aisles and shelves"""
        warehouse_path = f"/World/{warehouse_name}"

        # Create floor
        create_prim(
            prim_path=f"{warehouse_path}/Floor",
            prim_type="Plane",
            position=[0, 0, 0],
            scale=[size[0]/2, size[1]/2, 1]
        )

        # Create grid of shelves
        shelf_width = 1.0
        shelf_depth = 0.5
        shelf_height = 2.0
        aisle_width = 2.0

        rows = int(size[1] / (shelf_depth + aisle_width))
        cols = int(size[0] / (shelf_width + aisle_width))

        start_x = -size[0]/2 + shelf_width/2
        start_y = -size[1]/2 + shelf_depth/2

        for row in range(rows):
            for col in range(cols):
                if random.choice([True, False]):  # Randomly place shelves
                    x = start_x + col * (shelf_width + aisle_width)
                    y = start_y + row * (shelf_depth + aisle_width)

                    shelf_path = f"{warehouse_path}/Shelf_{row}_{col}"
                    create_prim(
                        prim_path=shelf_path,
                        prim_type="Cuboid",
                        position=[x, y, shelf_height/2],
                        scale=[shelf_width, shelf_depth, shelf_height]
                    )

    def generate_outdoor_environment(self, name="Outdoor", size=(50, 50)):
        """Generate an outdoor environment with terrain and objects"""
        env_path = f"/World/{name}"

        # Create terrain (simplified as a large plane with some variation)
        create_prim(
            prim_path=f"{env_path}/Ground",
            prim_type="Plane",
            position=[0, 0, 0],
            scale=[size[0]/2, size[1]/2, 1]
        )

        # Add some trees
        num_trees = random.randint(5, 15)
        for i in range(num_trees):
            x = random.uniform(-size[0]/2 + 5, size[0]/2 - 5)
            y = random.uniform(-size[1]/2 + 5, size[1]/2 - 5)

            tree_path = f"{env_path}/Tree_{i}"
            # In a real implementation, you would use actual tree models
            create_prim(
                prim_path=tree_path,
                prim_type="Cylinder",
                position=[x, y, 1.5],
                scale=[0.3, 0.3, 3.0]
            )

        # Add rocks/boulders
        num_rocks = random.randint(3, 8)
        for i in range(num_rocks):
            x = random.uniform(-size[0]/2 + 3, size[0]/2 - 3)
            y = random.uniform(-size[1]/2 + 3, size[1]/2 - 3)

            rock_size = random.uniform(0.5, 1.5)
            rock_path = f"{env_path}/Rock_{i}"
            create_prim(
                prim_path=rock_path,
                prim_type="Sphere",
                position=[x, y, rock_size/2],
                scale=[rock_size] * 3
            )

# Example usage
env_generator = ProceduralEnvironmentGenerator()

# Generate different types of environments
indoor_room = env_generator.generate_random_room("RandomIndoorRoom")
warehouse = env_generator.generate_warehouse_environment("RandomWarehouse")
outdoor_env = env_generator.generate_outdoor_environment("RandomOutdoor", size=(30, 30))
```

## Advanced Sensor Simulation

### Multi-sensor Fusion

```python
from omni.isaac.sensor import Camera
from omni.isaac.range_sensor import _range_sensor
from omni.isaac.core.sensors import ImuSensor
import numpy as np

class MultiSensorFusion:
    def __init__(self, robot):
        self.robot = robot
        self.sensors = {}
        self.fusion_data = {}

    def add_camera(self, name, prim_path, position, resolution=(640, 480)):
        """Add a camera sensor"""
        camera = Camera(
            prim_path=prim_path,
            frequency=30,
            resolution=resolution
        )
        camera.set_translation(position)
        self.sensors[name] = {
            "type": "camera",
            "sensor": camera,
            "position": position
        }

    def add_lidar(self, name, prim_path, position):
        """Add a LiDAR sensor"""
        lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
        lidar = lidar_interface.new_lidar(
            prim_path,
            translation=position,
            min_range=0.1,
            max_range=25.0,
            horizontal_samples=1080,
            vertical_samples=64,
            horizontal_fov=360.0,
            vertical_fov=30.0
        )
        self.sensors[name] = {
            "type": "lidar",
            "sensor": lidar,
            "interface": lidar_interface,
            "position": position
        }

    def add_imu(self, name, prim_path, position):
        """Add an IMU sensor"""
        imu = ImuSensor(
            prim_path=prim_path,
            name=name,
            position=position
        )
        self.sensors[name] = {
            "type": "imu",
            "sensor": imu,
            "position": position
        }

    def get_fused_sensor_data(self):
        """Get and fuse data from all sensors"""
        fused_data = {}

        for name, sensor_info in self.sensors.items():
            sensor_type = sensor_info["type"]

            if sensor_type == "camera":
                rgb_data = sensor_info["sensor"].get_rgb()
                depth_data = sensor_info["sensor"].get_depth()
                fused_data[name] = {
                    "rgb": rgb_data,
                    "depth": depth_data,
                    "timestamp": self._get_current_time()
                }

            elif sensor_type == "lidar":
                lidar_data = sensor_info["interface"].get_linear_depth_data(
                    sensor_info["sensor"].GetPath().pathString
                )
                fused_data[name] = {
                    "scan": lidar_data,
                    "timestamp": self._get_current_time()
                }

            elif sensor_type == "imu":
                linear_acc = sensor_info["sensor"].get_linear_acceleration()
                angular_vel = sensor_info["sensor"].get_angular_velocity()
                orientation = sensor_info["sensor"].get_orientation()
                fused_data[name] = {
                    "linear_acceleration": linear_acc,
                    "angular_velocity": angular_vel,
                    "orientation": orientation,
                    "timestamp": self._get_current_time()
                }

        # Perform sensor fusion
        self.fusion_data = self._perform_fusion(fused_data)
        return self.fusion_data

    def _perform_fusion(self, raw_data):
        """Perform basic sensor fusion"""
        fused_result = {}

        # Example: Combine camera depth and LiDAR for better depth estimation
        if "camera" in raw_data and "lidar" in raw_data:
            camera_depth = raw_data["camera"]["depth"]
            lidar_scan = raw_data["lidar"]["scan"]

            # Simple fusion: use LiDAR for long-range, camera for short-range
            fused_depth = self._fuse_depth_data(camera_depth, lidar_scan)
            fused_result["fused_depth"] = fused_depth

        # Example: Combine IMU data for better pose estimation
        if "imu" in raw_data:
            imu_data = raw_data["imu"]
            estimated_pose = self._integrate_imu_data(imu_data)
            fused_result["estimated_pose"] = estimated_pose

        return fused_result

    def _fuse_depth_data(self, camera_depth, lidar_scan):
        """Fuse camera and LiDAR depth data"""
        # This is a simplified example
        # In practice, you would use more sophisticated algorithms
        fused_depth = np.copy(camera_depth)

        # Where LiDAR data is available and reliable, use it
        # Otherwise, fall back to camera data
        return fused_depth

    def _integrate_imu_data(self, imu_data):
        """Integrate IMU data for pose estimation"""
        # Simplified integration
        # In practice, use proper sensor fusion algorithms like Kalman filters
        return {
            "position": [0, 0, 0],  # Would be integrated from acceleration
            "orientation": imu_data["orientation"]
        }

    def _get_current_time(self):
        """Get current simulation time"""
        from omni.isaac.core import World
        world = World()
        return world.current_time

# Example usage
multi_sensor = MultiSensorFusion(robot)

# Add multiple sensors to the robot
multi_sensor.add_camera("front_camera", "/World/Robot/Camera", [0.2, 0, 0.1])
multi_sensor.add_lidar("3d_lidar", "/World/Robot/Lidar", [0.1, 0, 0.3])
multi_sensor.add_imu("body_imu", "/World/Robot/IMU", [0, 0, 0.2])

# Get fused sensor data
fused_data = multi_sensor.get_fused_sensor_data()
```

## Performance Optimization

### Simulation Optimization Techniques

```python
from omni.isaac.core.utils.stage import set_stage_units
from omni.isaac.core import World
import carb

class SimulationOptimizer:
    def __init__(self, world):
        self.world = world
        self.optimization_settings = {
            "enable_gpu_physics": True,
            "use_variable_timestep": True,
            "lod_bias": 0.0,
            "culling_enabled": True
        }

    def optimize_for_training(self):
        """Optimize simulation for AI training"""
        # Reduce visual quality for faster simulation
        self._set_render_quality("low")

        # Increase physics substeps for stability
        carb.settings.get_settings().set("/physics/solverPositionIterations", 4)
        carb.settings.get_settings().set("/physics/solverVelocityIterations", 1)

        # Enable GPU physics if available
        carb.settings.get_settings().set("/physics/cudaDevice", 0)
        carb.settings.get_settings().set("/physics/useGPU", True)

    def optimize_for_visualization(self):
        """Optimize simulation for visualization"""
        # Increase visual quality
        self._set_render_quality("high")

        # Reduce physics complexity for smoother visualization
        carb.settings.get_settings().set("/physics/solverPositionIterations", 8)
        carb.settings.get_settings().set("/physics/solverVelocityIterations", 4)

    def _set_render_quality(self, quality_level):
        """Set rendering quality level"""
        if quality_level == "low":
            carb.settings.get_settings().set("/rtx-defaults/ambientOcclusion/raySteps", 4)
            carb.settings.get_settings().set("/rtx-defaults/pathtracingSamplesPerFrame", 1)
        elif quality_level == "high":
            carb.settings.get_settings().set("/rtx-defaults/ambientOcclusion/raySteps", 16)
            carb.settings.get_settings().set("/rtx-defaults/pathtracingSamplesPerFrame", 16)

    def enable_culling(self):
        """Enable view frustum culling for performance"""
        carb.settings.get_settings().set("/app/performer/enableFrustumCulling", True)

    def set_physics_timestep(self, timestep):
        """Set physics timestep for the simulation"""
        from omni.isaac.core.utils.simulation import SimulationApp
        # This would typically be set during simulation app initialization
        pass

    def batch_multiple_environments(self):
        """Setup for batched training environments"""
        # This involves creating multiple identical environments
        # in the same simulation for parallel training
        pass

# Example usage
optimizer = SimulationOptimizer(world)
optimizer.optimize_for_training()
```

## Practical Exercise

Implement a complete AI training pipeline with:

1. Domain randomization for lighting, materials, and physics
2. A procedural environment generator
3. Multi-sensor robot with camera and LiDAR
4. Reinforcement learning integration
5. Performance optimization for training speed

## Summary

Isaac Sim's advanced features provide powerful capabilities for AI development in robotics. Domain randomization, reinforcement learning integration, and procedural environment generation make it an ideal platform for developing robust AI models that can transfer from simulation to reality.

## Next Steps

The next chapter will focus on Isaac ROS Navigation, exploring how to implement navigation stacks, path planning, and autonomous navigation in Isaac Sim environments.
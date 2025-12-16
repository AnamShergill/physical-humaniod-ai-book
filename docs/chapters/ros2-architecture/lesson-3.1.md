---
title: Nodes, Topics, Services, Actions
description: Understanding the fundamental building blocks of ROS 2 architecture
sidebar_label: Lesson 3.1 - Nodes, Topics, Services, Actions
---

# Nodes, Topics, Services, Actions

## Learning Objectives

After completing this lesson, you will be able to:
- Define and distinguish between nodes, topics, services, and actions in ROS 2
- Create and implement basic ROS 2 nodes for communication
- Design topic-based and service-based communication patterns
- Choose appropriate communication mechanisms for different use cases

## Introduction

ROS 2 (Robot Operating System 2) provides a flexible framework for building robot applications through a distributed computing architecture. The core communication patterns in ROS 2 include nodes for computation, topics for asynchronous data streaming, services for request-response communication, and actions for goal-oriented, long-running operations.

## Core Concepts

### Nodes
Nodes are the fundamental computational units in ROS 2. Each node runs a specific task and communicates with other nodes through topics, services, and actions. Nodes are implemented as processes that perform computation and can be written in multiple languages (C++, Python, etc.).

### Topics
Topics provide asynchronous, many-to-many communication through a publish-subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics. This decouples nodes in time and space, allowing for flexible system design.

### Services
Services provide synchronous, request-response communication. A client sends a request to a service and waits for a response. This is useful for operations that require immediate feedback or completion confirmation.

### Actions
Actions are designed for long-running operations that may take significant time to complete. They support goal setting, feedback during execution, and result reporting. Actions are ideal for navigation, manipulation, or any task that requires monitoring progress.

## Mental Models

### The Communication Spectrum
ROS 2 communication patterns form a spectrum from simple data streaming (topics) to complex goal-oriented operations (actions). Understanding when to use each pattern is crucial for effective system design.

### Distributed Architecture Pattern
ROS 2's distributed architecture allows nodes to run on different machines while maintaining seamless communication, enabling scalable and fault-tolerant robot systems.

## Code Examples

### Example 1: Basic ROS 2 Node
Creating a simple ROS 2 node that publishes sensor data:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        self.get_logger().info('Sensor Node Started')

    def publish_sensor_data(self):
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = [0.1, 0.2, 0.3]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint states: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    sensor_node = SensorNode()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Service Server and Client
Implementing a service for calculating inverse kinematics:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Compute

class IKService(Node):
    def __init__(self):
        super().__init__('ik_service')
        self.srv = self.create_service(Compute, 'inverse_kinematics', self.ik_callback)

    def ik_callback(self, request, response):
        # Simplified inverse kinematics calculation
        # In practice, this would use more complex algorithms
        x, y, z = request.x, request.y, request.z

        # Calculate joint angles for desired end-effector position
        joint1 = math.atan2(y, x)
        distance = math.sqrt(x*x + y*y)
        joint2 = math.atan2(z, distance)

        response.result = [joint1, joint2, 0.0]  # Simplified result
        self.get_logger().info(f'IK: {request.x}, {request.y}, {request.z} -> {response.result}')
        return response

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.cli = self.create_client(Compute, 'inverse_kinematics')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Compute.Request()

    def send_request(self, x, y, z):
        self.req.x = x
        self.req.y = y
        self.req.z = z
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    import math
    rclpy.init(args=args)

    client = IKClient()
    response = client.send_request(1.0, 0.5, 0.8)
    if response:
        print(f'Joint angles: {response.result}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3: Action Server and Client
Creating an action for robot navigation:

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from nav_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        self.get_logger().info('Received navigation goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing navigation goal...')

        target_pose = goal_handle.request.pose
        current_pose = PoseStamped()  # Simplified - would get actual pose in real implementation

        # Simulate navigation progress
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        for i in range(100):  # Simulate progress
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.error_code = 1
                return result

            # Calculate distance to goal
            dist = math.sqrt(
                (target_pose.pose.position.x - current_pose.pose.position.x)**2 +
                (target_pose.pose.position.y - current_pose.pose.position.y)**2
            )

            feedback_msg.current_pose = current_pose
            feedback_msg.distance_remaining = dist
            goal_handle.publish_feedback(feedback_msg)

            # Simulate movement
            await rclpy.asyncio.sleep(0.1)

            if dist < 0.1:  # Reached goal
                goal_handle.succeed()
                result.result = current_pose
                self.get_logger().info('Navigation succeeded')
                return result

        goal_handle.succeed()
        result.result = current_pose
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = NavigationActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Simulation Exercises

### Exercise 1: Node Communication
- **Objective**: Create two ROS 2 nodes that communicate via topics
- **Requirements**: ROS 2 Foxy/Humble, Python
- **Steps**:
  1. Create a publisher node that publishes sensor data
  2. Create a subscriber node that receives and processes the data
  3. Test the communication between nodes
  4. Monitor the data flow using ROS 2 tools
- **Expected Outcome**: Understanding of topic-based communication in ROS 2

### Exercise 2: Service Implementation
- **Objective**: Implement a service for robot control
- **Requirements**: ROS 2 Foxy/Humble, Python
- **Steps**:
  1. Define a custom service message
  2. Create a service server that performs a specific task
  3. Create a service client that sends requests
  4. Test the request-response communication
- **Expected Outcome**: Understanding of service-based communication in ROS 2

### Exercise 3: Action Implementation
- **Objective**: Implement an action for long-running robot task
- **Requirements**: ROS 2 Foxy/Humble, Python
- **Steps**:
  1. Define a custom action message
  2. Create an action server with goal, feedback, and result
  3. Create an action client that sends goals and monitors progress
  4. Test the goal-oriented communication pattern
- **Expected Outcome**: Understanding of action-based communication for complex tasks

## Summary

ROS 2 provides a comprehensive communication architecture with nodes as computational units and four primary communication patterns: topics for asynchronous data streaming, services for request-response interactions, actions for goal-oriented operations, and parameters for configuration. Understanding these patterns is essential for designing effective robot applications with proper decoupling, scalability, and maintainability.

## Key Terms

- **Node**: A process performing computation in ROS 2
- **Topic**: A stream of messages passed between nodes using publish-subscribe pattern
- **Service**: Synchronous request-response communication pattern
- **Action**: Goal-oriented communication pattern with feedback and result
- **Publisher**: Node that sends messages to a topic
- **Subscriber**: Node that receives messages from a topic
- **Client**: Node that sends requests to a service
- **Server**: Node that responds to service requests

## Next Steps

Continue to Lesson 3.2: "Launch Files and Parameters" to explore how ROS 2 systems are configured and launched in organized ways.
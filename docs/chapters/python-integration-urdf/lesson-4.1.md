---
title: rclpy Python Client Library
description: Understanding Python integration with ROS 2 using the rclpy client library
sidebar_label: Lesson 4.1 - rclpy Python Client Library
---

import LessonHeader from '@site/src/components/LessonHeader';
import CalloutBlock from '@site/src/components/CalloutBlock';
import QuizBlock from '@site/src/components/QuizBlock';
import AIChatPanel from '@site/src/components/AIChatPanel';
import Breadcrumb from '@site/src/components/Breadcrumb';

<Breadcrumb items={[
  { label: 'Chapters', href: '/docs/chapters/python-integration-urdf' },
  { label: 'Python Integration & URDF', href: '/docs/chapters/python-integration-urdf' },
  { label: 'rclpy Python Client Library' }
]} />

<LessonHeader
  title="rclpy Python Client Library"
  subtitle="Understanding Python integration with ROS 2 using the rclpy client library"
  chapter="4"
  lessonNumber="4.1"
  progress={25}
/>

# rclpy Python Client Library

## Learning Objectives

After completing this lesson, you will be able to:
- Use the rclpy library to create ROS 2 nodes in Python
- Implement publishers, subscribers, services, and actions in Python
- Apply Python-specific patterns and best practices for ROS 2 development
- Integrate Python libraries with ROS 2 systems

## Introduction

rclpy is the Python client library for ROS 2 that provides the interface between Python applications and the ROS 2 middleware. It enables Python developers to create ROS 2 nodes, publish and subscribe to topics, provide and call services, and implement actions. Understanding rclpy is essential for leveraging Python's rich ecosystem in robot applications.

## Core Concepts

### rclpy Architecture
rclpy provides a Python binding to the ROS Client Library (rcl) and offers an object-oriented interface for ROS 2 concepts like nodes, publishers, subscribers, services, and actions.

### Python-Specific Features
Python's dynamic nature and rich ecosystem make it ideal for rapid prototyping, data processing, and machine learning integration with ROS 2 systems.

### Async Support
rclpy provides both synchronous and asynchronous execution models, allowing for efficient handling of concurrent operations and non-blocking I/O.

## Mental Models

### Python-ROS 2 Bridge
rclpy acts as a bridge between Python's ecosystem and ROS 2's distributed architecture, enabling seamless integration of Python libraries and tools with robot systems.

### Asynchronous Event Loop
Understanding how rclpy integrates with Python's asyncio event loop for handling multiple concurrent operations efficiently.

## Code Examples

### Example 1: Basic Node with Publishers and Subscribers
Creating a Python node that processes sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class SensorProcessingNode(Node):
    def __init__(self):
        super().__init__('sensor_processing_node')

        # Create subscriber for laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Create publisher for velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for periodic processing
        self.timer = self.create_timer(0.1, self.process_data)

        # Data storage
        self.latest_scan = None
        self.obstacle_detected = False

        self.get_logger().info('Sensor processing node initialized')

    def laser_callback(self, msg):
        """Process incoming laser scan data"""
        self.latest_scan = msg
        self.get_logger().debug(f'Received scan with {len(msg.ranges)} points')

    def process_data(self):
        """Process sensor data and generate commands"""
        if self.latest_scan is not None:
            # Simple obstacle detection
            min_distance = min([r for r in self.latest_scan.ranges if 0.1 < r < 10.0], default=float('inf'))

            cmd = Twist()
            if min_distance < 0.5:  # Obstacle too close
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5  # Turn
                self.obstacle_detected = True
            else:
                cmd.linear.x = 0.5
                cmd.angular.z = 0.0
                self.obstacle_detected = False

            self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Service Server and Client in Python
Implementing a service for mathematical operations:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from example_interfaces.srv import AddTwoInts

class MathService(Node):
    def __init__(self):
        super().__init__('math_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('Math service server started')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response

class MathClient(Node):
    def __init__(self):
        super().__init__('math_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)

    # Create client node
    client = MathClient()

    # Send request
    future = client.send_request(42, 38)

    # Spin until future is complete
    rclpy.spin_until_future_complete(client, future)

    # Process response
    result = future.result()
    if result:
        client.get_logger().info(f'Result: {result.sum}')
    else:
        client.get_logger().error('Service call failed')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3: Asynchronous Processing with rclpy
Using async/await for concurrent operations:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor

class AsyncProcessingNode(Node):
    def __init__(self):
        super().__init__('async_processing_node')

        # Publishers
        self.status_pub = self.create_publisher(String, 'async_status', 10)
        self.result_pub = self.create_publisher(String, 'async_result', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )

        # Timer for async operations
        self.timer = self.create_timer(1.0, self.start_async_operations)

        # Thread pool for CPU-intensive tasks
        self.executor = ThreadPoolExecutor(max_workers=4)

        self.get_logger().info('Async processing node initialized')

    def image_callback(self, msg):
        """Handle incoming image data asynchronously"""
        self.get_logger().info(f'Received image: {msg.width}x{msg.height}')

    def start_async_operations(self):
        """Start async operations in background"""
        # Create a task that runs concurrently
        future = asyncio.run_coroutine_threadsafe(
            self.perform_async_task(),
            self.executor_loop
        )
        future.add_done_callback(self.async_task_completed)

    async def perform_async_task(self):
        """Simulate async processing task"""
        status_msg = String()
        status_msg.data = 'Starting async task...'
        self.status_pub.publish(status_msg)

        # Simulate async work
        await asyncio.sleep(2.0)

        # Simulate CPU-intensive work in thread pool
        result = await self.run_in_executor(self.cpu_intensive_task)

        result_msg = String()
        result_msg.data = f'Async task completed: {result}'
        self.result_pub.publish(result_msg)

        return result

    async def run_in_executor(self, func, *args):
        """Run CPU-intensive function in thread pool"""
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(self.executor, func, *args)

    def cpu_intensive_task(self):
        """Simulate CPU-intensive task"""
        import time
        time.sleep(1)  # Simulate computation
        return "Processed data"

    def async_task_completed(self, future):
        """Callback when async task completes"""
        try:
            result = future.result()
            self.get_logger().info(f'Async task result: {result}')
        except Exception as e:
            self.get_logger().error(f'Async task failed: {e}')

def main(args=None):
    rclpy.init(args=args)

    # Get the rclpy context's executor
    node = AsyncProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.executor.shutdown(wait=True)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Simulation Exercises

### Exercise 1: Python Node Development
- **Objective**: Create a Python node that processes sensor data
- **Requirements**: ROS 2 Foxy/Humble, Python, rclpy
- **Steps**:
  1. Create a node that subscribes to sensor data
  2. Implement data processing logic using NumPy
  3. Publish processed results to output topics
  4. Test the node with simulated sensor data
- **Expected Outcome**: Understanding of Python node development with rclpy

### Exercise 2: Service Implementation in Python
- **Objective**: Implement a service for data processing
- **Requirements**: ROS 2 Foxy/Humble, Python, rclpy
- **Steps**:
  1. Define a custom service message
  2. Create a service server that processes requests
  3. Create a service client that sends requests
  4. Test the service with various inputs
- **Expected Outcome**: Understanding of service implementation in Python

### Exercise 3: Async Operations with rclpy
- **Objective**: Implement concurrent operations using Python's async features
- **Requirements**: ROS 2 Foxy/Humble, Python, rclpy, asyncio
- **Steps**:
  1. Create a node that performs multiple operations concurrently
  2. Use async/await for non-blocking operations
  3. Integrate with thread pools for CPU-intensive tasks
  4. Test performance improvements over synchronous approach
- **Expected Outcome**: Understanding of asynchronous programming with rclpy

## Summary

The rclpy library provides comprehensive Python integration with ROS 2, enabling developers to leverage Python's rich ecosystem for robot applications. It supports all core ROS 2 concepts including nodes, publishers, subscribers, services, and actions, with both synchronous and asynchronous execution models. Understanding rclpy is essential for effective Python-based robot development.

## Key Terms

- **rclpy**: Python client library for ROS 2
- **Node**: The basic execution unit in ROS 2
- **Publisher**: Component that sends messages to topics
- **Subscriber**: Component that receives messages from topics
- **Service**: Synchronous request-response communication
- **Action**: Goal-oriented communication with feedback
- **Async/Await**: Python keywords for asynchronous programming
- **ThreadPoolExecutor**: Python class for managing thread pools

## Next Steps

Continue to Lesson 4.2: "Creating and Using URDF for Humanoids" to explore robot modeling with URDF.
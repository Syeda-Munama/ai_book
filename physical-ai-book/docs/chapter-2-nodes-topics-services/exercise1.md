# Exercise 1: Publisher/Subscriber System for Humanoid Joint Control

## Objective

In this exercise, you will implement a complete publisher/subscriber system for controlling humanoid joints using ROS 2. This system will allow you to publish joint position commands and subscribe to joint state feedback.

## Prerequisites

- ROS 2 Humble installed on Ubuntu 22.04
- Basic Python programming knowledge
- Understanding of ROS 2 nodes, topics, and messages

## Step 1: Create a ROS 2 Package

First, create a new ROS 2 package for our joint control system:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create a new workspace (if you don't have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create the package
ros2 pkg create --build-type ament_python joint_control_pkg
cd joint_control_pkg
```

## Step 2: Create the Publisher Node

Create a publisher node that will send joint position commands:

```python
# In ~/ros2_ws/src/joint_control_pkg/joint_control_pkg/joint_command_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'joint_commands', 10)

        # Timer to publish commands at 10 Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64MultiArray()

        # Simulate oscillating joint positions for 6 joints
        # This could represent a walking gait or other periodic motion
        joint_positions = [
            math.sin(self.i * 0.1) * 0.5,      # Joint 1: Hip
            math.cos(self.i * 0.1) * 0.3,      # Joint 2: Knee
            math.sin(self.i * 0.1 + 1.0) * 0.4, # Joint 3: Ankle
            math.sin(self.i * 0.1 + 0.5) * 0.6, # Joint 4: Shoulder
            math.cos(self.i * 0.1 + 0.5) * 0.5, # Joint 5: Elbow
            math.sin(self.i * 0.1 + 1.5) * 0.3  # Joint 6: Wrist
        ]

        msg.data = joint_positions
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint commands: {joint_positions}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    joint_command_publisher = JointCommandPublisher()

    try:
        rclpy.spin(joint_command_publisher)
    except KeyboardInterrupt:
        pass

    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 3: Create the Subscriber Node

Create a subscriber node that will receive and process joint state feedback:

```python
# In ~/ros2_ws/src/joint_control_pkg/joint_control_pkg/joint_state_subscriber.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')

        # Subscribe to joint commands
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_command_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publish joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Define joint names
        self.joint_names = [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle'
        ]

    def joint_command_callback(self, msg):
        self.get_logger().info(f'Received joint commands: {msg.data}')

        # Process the commands and simulate joint state response
        # In a real robot, this would interface with actual joint controllers
        joint_positions = msg.data

        # Create and publish joint state message
        joint_state_msg = JointState()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = joint_positions
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = 'base_link'

        self.joint_state_publisher.publish(joint_state_msg)
        self.get_logger().info(f'Published joint states: {joint_positions}')

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()

    try:
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        pass

    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 4: Update setup.py

Update the package's setup.py file to make the scripts executable:

```python
# In ~/ros2_ws/src/joint_control_pkg/setup.py

from setuptools import find_packages, setup

package_name = 'joint_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Package for humanoid joint control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_command_publisher = joint_control_pkg.joint_command_publisher:main',
            'joint_state_subscriber = joint_control_pkg.joint_state_subscriber:main',
        ],
    },
)
```

## Step 5: Test the System

Build and run your publisher/subscriber system:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the package
cd ~/ros2_ws
colcon build --packages-select joint_control_pkg

# Source the workspace
source install/setup.bash

# Terminal 1: Run the publisher
ros2 run joint_control_pkg joint_command_publisher

# Terminal 2: Run the subscriber
ros2 run joint_control_pkg joint_state_subscriber
```

## Expected Results

You should see the publisher sending joint position commands and the subscriber receiving them, then publishing the corresponding joint states. The system simulates a humanoid robot with 6 joints (3 on each leg) that move in coordinated patterns.

## Challenges

1. Modify the publisher to send different movement patterns (e.g., walking gait, arm waving).
2. Add velocity and effort values to the JointState message.
3. Implement a simple PID controller in the subscriber that adjusts the joint positions based on error feedback.

## Solution Verification

Your system should successfully:
- Publish joint commands at a regular interval
- Subscribe to those commands and process them
- Publish corresponding joint states based on the commands
- Log messages showing the data flow between nodes

## Next Steps

This exercise demonstrates the core communication patterns used in humanoid robotics. In the next chapter, we'll build on this foundation to create more complex ROS 2 packages with Python and explore URDF for humanoid models.
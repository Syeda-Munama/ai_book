# Exercise 2: Building a ROS 2 Package with URDF for Humanoid Control

## Objective

In this exercise, you will create a complete ROS 2 package that combines Python nodes, launch files, parameters, and URDF to control a simple humanoid robot model. This will demonstrate the integration of all concepts learned in this chapter.

## Prerequisites

- ROS 2 Humble installed on Ubuntu 22.04
- Basic Python programming knowledge
- Understanding of ROS 2 packages, launch files, parameters, and URDF

## Step 1: Create the ROS 2 Package

First, create a new ROS 2 package for our humanoid robot:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create a new workspace (if you don't have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create the package
ros2 pkg create --build-type ament_python humanoid_robot_pkg --dependencies rclpy std_msgs sensor_msgs geometry_msgs builtin_interfaces
cd humanoid_robot_pkg
```

## Step 2: Create the URDF Model

Create a URDF file for a simple humanoid model:

```xml
<!-- In ~/ros2_ws/src/humanoid_robot_pkg/humanoid_robot_pkg/simple_humanoid.urdf -->
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base torso -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right Arm -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

## Step 3: Create the Robot State Publisher Node

Create a Python script to publish the robot state:

```python
# In ~/ros2_ws/src/humanoid_robot_pkg/humanoid_robot_pkg/robot_state_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Declare parameters
        self.declare_parameter('publish_frequency', 50.0)
        self.declare_parameter('robot_description', '')

        self.publish_freq = self.get_parameter('publish_frequency').value

        # Create joint state publisher
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create timer
        timer_period = 1.0 / self.publish_freq
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize joint positions
        self.time = 0.0

        # Read robot description parameter
        robot_desc = self.get_parameter('robot_description').value
        if not robot_desc:
            self.get_logger().warn('No robot_description parameter set')

    def timer_callback(self):
        # Create joint state message
        msg = JointState()
        msg.name = [
            'neck_joint',
            'left_shoulder_joint',
            'right_shoulder_joint'
        ]

        # Create oscillating joint positions
        neck_pos = 0.2 * math.sin(self.time * 0.5)
        left_arm_pos = 0.5 * math.sin(self.time * 0.7)
        right_arm_pos = 0.5 * math.cos(self.time * 0.7)

        msg.position = [neck_pos, left_arm_pos, right_arm_pos]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]

        # Set timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Publish joint states
        self.joint_state_publisher.publish(msg)

        # Broadcast transforms
        self.broadcast_transforms(msg)

        self.time += 1.0 / self.publish_freq

    def broadcast_transforms(self, joint_state):
        # This is a simplified version - in practice, you'd calculate transforms
        # based on joint positions using kinematics
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 4: Create a Joint Controller Node

Create a simple joint controller:

```python
# In ~/ros2_ws/src/humanoid_robot_pkg/humanoid_robot_pkg/joint_controller.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Time
import time

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Declare parameters
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('default_joint_positions', [0.0, 0.0, 0.0])

        self.control_freq = self.get_parameter('control_frequency').value
        self.default_positions = self.get_parameter('default_joint_positions').value

        # Create publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(Float64MultiArray, 'joint_commands', 10)

        # Create timer for control loop
        timer_period = 1.0 / self.control_freq
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info('Joint Controller initialized')

    def control_loop(self):
        # Create command message with oscillating positions
        msg = Float64MultiArray()

        # Generate time-varying joint commands
        current_time = self.get_clock().now().nanoseconds / 1e9
        commands = [
            0.3 * (current_time % 2.0),  # neck
            0.4 * (current_time % 1.5),  # left arm
            -0.4 * (current_time % 1.5)  # right arm
        ]

        msg.data = commands
        self.joint_cmd_publisher.publish(msg)

        self.get_logger().info(f'Published joint commands: {commands}')

def main(args=None):
    rclpy.init(args=args)
    node = JointController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 5: Create the Launch File

Create a launch file to start the complete system:

```python
# In ~/ros2_ws/src/humanoid_robot_pkg/humanoid_robot_pkg/humanoid_system.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('humanoid_robot_pkg')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        # Robot state publisher node
        Node(
            package='humanoid_robot_pkg',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'publish_frequency': 50.0},
                {'robot_description': ''}  # Will be set from URDF file
            ],
            output='screen'
        ),

        # Joint controller node
        Node(
            package='humanoid_robot_pkg',
            executable='joint_controller',
            name='joint_controller',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'control_frequency': 10.0},
                {'default_joint_positions': [0.0, 0.0, 0.0]}
            ],
            output='screen'
        ),
    ])
```

## Step 6: Update setup.py

Update the package's setup.py file:

```python
# In ~/ros2_ws/src/humanoid_robot_pkg/setup.py

from setuptools import find_packages, setup

package_name = 'humanoid_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add URDF file to data files
        (os.path.join('share', package_name, 'urdf'),
         [os.path.join('humanoid_robot_pkg', 'simple_humanoid.urdf')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Package for humanoid robot control with ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state_publisher = humanoid_robot_pkg.robot_state_publisher:main',
            'joint_controller = humanoid_robot_pkg.joint_controller:main',
        ],
    },
)
```

## Step 7: Test the System

Build and run your humanoid robot system:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the package
cd ~/ros2_ws
colcon build --packages-select humanoid_robot_pkg

# Source the workspace
source install/setup.bash

# Run the system
ros2 launch humanoid_robot_pkg humanoid_system.launch.py
```

## Expected Results

You should see both nodes running:
- The robot state publisher publishing joint states
- The joint controller publishing commands
- The system should be ready to integrate with visualization tools like RViz2

## Challenges

1. Add more joints to the URDF model (elbows, wrists, hips, knees, ankles)
2. Implement a more sophisticated control algorithm that coordinates multiple joints
3. Add a parameter server to dynamically adjust joint limits and control parameters
4. Create a service that allows changing the robot's behavior pattern

## Solution Verification

Your system should successfully:
- Create a proper ROS 2 package with correct structure
- Define a valid URDF model for a humanoid robot
- Launch multiple nodes with parameters
- Publish joint states at the specified frequency
- Demonstrate the integration of packages, launch files, parameters, and URDF

## Next Steps

This exercise combines all the concepts from this chapter. In the next chapter, we'll integrate all ROS 2 concepts into a comprehensive humanoid robot system that demonstrates the complete "robotic nervous system" approach.
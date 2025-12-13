# ROS 2 Nodes

## Introduction to Nodes

A node is a process that performs computation in ROS 2. Nodes are the fundamental building blocks of a ROS program, and they can be written in different programming languages (C++, Python, etc.). In the context of humanoid robotics, nodes can represent different subsystems such as joint controllers, sensor processors, or high-level decision makers.

## Creating Nodes

In ROS 2, nodes are created by subclassing the `rclpy.Node` class in Python or using the appropriate base class in other languages. Each node should have a unique name within the ROS graph to prevent conflicts.

### Basic Node Structure in Python

```python
import rclpy
from rclpy.node import Node

class HumanoidJointController(Node):
    def __init__(self):
        super().__init__('humanoid_joint_controller')
        self.get_logger().info('Humanoid Joint Controller node initialized')

        # Initialize any required components here
        self.joint_positions = {}

    def control_joint(self, joint_name, target_position):
        # Implementation for controlling a specific joint
        self.get_logger().info(f'Moving {joint_name} to {target_position}')
```

## Node Lifecycle

ROS 2 nodes have a well-defined lifecycle that includes:
- **Unconfigured**: Node is created but not configured
- **Inactive**: Node is configured but not active
- **Active**: Node is fully operational
- **Finalized**: Node is shutting down

## Communication Patterns in Nodes

Nodes communicate with each other using various patterns:
- **Publish-Subscribe**: For asynchronous data distribution
- **Request-Response**: For synchronous service calls
- **Action-Based**: For long-running tasks with feedback

## Best Practices for Node Design

When designing nodes for humanoid robotics:
- Keep nodes focused on a single responsibility
- Use meaningful node names that reflect their function
- Implement proper error handling and logging
- Consider resource usage and real-time constraints
- Design nodes to be reusable across different robot platforms

## Node Management

ROS 2 provides tools for managing nodes:
- Launch files for starting multiple nodes together
- Parameters for configuring node behavior
- Services for runtime management
- Monitoring tools for debugging and diagnostics

In the next section, we'll explore topics and message passing in detail.
# Parameters

## Introduction to Parameters

Parameters in ROS 2 provide a way to configure nodes at runtime. They allow you to change node behavior without recompiling code, making your robot systems more flexible and configurable.

## Parameter Basics

Parameters are key-value pairs that can be set at different levels:
- Default values defined in the node code
- Values set via launch files
- Values set via command line
- Values set programmatically at runtime

## Declaring Parameters in Code

To use parameters in your nodes, you need to declare them:

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')

        # Declare parameters with default values
        self.declare_parameter('wheel_radius', 0.1)  # meters
        self.declare_parameter('max_velocity', 1.0)  # m/s
        self.declare_parameter('robot_name', 'my_robot')

        # Get parameter values
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.robot_name = self.get_parameter('robot_name').value

        self.get_logger().info(f'Robot {self.robot_name} initialized with wheel radius {self.wheel_radius}')

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()

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

## Setting Parameters in Launch Files

Parameters can be set in launch files:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='my_robot_node',
            name='my_robot_node',
            parameters=[
                {'wheel_radius': 0.15},  # Override default value
                {'max_velocity': 2.0},
                {'robot_name': 'humanoid_robot'},
            ],
        ),
    ])
```

## Parameter Descriptions

You can provide descriptions and constraints for parameters:

```python
from rclpy.parameter import ParameterDescriptor
from rclpy.qos import qos_profile_parameters

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')

        # Declare parameter with description and constraints
        param_desc = ParameterDescriptor(
            description='Radius of the robot wheels in meters',
            additional_constraints='Must be positive'
        )

        self.declare_parameter(
            'wheel_radius',
            0.1,
            descriptor=param_desc
        )
```

## Parameter Callbacks

You can set up callbacks to handle parameter changes:

```python
from rcl_interfaces.msg import SetParametersResult

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')

        self.declare_parameter('max_velocity', 1.0)
        self.max_velocity = self.get_parameter('max_velocity').value

        # Set callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.value > 0:
                self.max_velocity = param.value
                self.get_logger().info(f'Max velocity updated to {self.max_velocity}')
                return SetParametersResult(successful=True)

        return SetParametersResult(successful=False)
```

## Command Line Parameter Setting

Parameters can be set from the command line:

```bash
ros2 run my_robot_package my_robot_node --ros-args -p wheel_radius:=0.2 -p max_velocity:=3.0
```

## Parameter Files

Parameters can be loaded from YAML files:

```yaml
my_robot_node:
  ros__parameters:
    wheel_radius: 0.15
    max_velocity: 2.0
    robot_name: "humanoid_robot"
    sensors:
      lidar_enabled: true
      camera_enabled: false
```

And used in launch files:

```python
Node(
    package='my_robot_package',
    executable='my_robot_node',
    name='my_robot_node',
    parameters=['path/to/params.yaml'],
),
```

## Best Practices

- Use descriptive parameter names
- Provide meaningful default values
- Validate parameter values in callbacks
- Document parameters with descriptions
- Group related parameters logically
- Use parameter files for complex configurations
- Consider security implications for parameters that affect system behavior

Parameters are crucial for making your humanoid robot systems configurable and adaptable to different environments and requirements.
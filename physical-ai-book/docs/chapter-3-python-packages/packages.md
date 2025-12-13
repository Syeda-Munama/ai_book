# Building ROS 2 Packages with Python (rclpy)

## Introduction to ROS 2 Packages

A ROS 2 package is a reusable, shareable unit of software that contains nodes, libraries, and other resources. Packages are the basic building blocks of ROS 2 software and provide a way to organize code, share functionality, and manage dependencies.

## Creating a ROS 2 Package

To create a new ROS 2 package, use the `ros2 pkg create` command:

```bash
ros2 pkg create --build-type ament_python my_robot_package
```

This command creates a new Python package with the necessary structure and configuration files.

## Package Structure

A typical ROS 2 Python package includes:

- `package.xml`: Metadata about the package (dependencies, maintainer, etc.)
- `setup.py`: Python setuptools configuration
- `setup.cfg`: Installation configuration
- `my_robot_package/`: Source code directory (same name as package)
  - `__init__.py`: Makes the directory a Python package
  - `my_node.py`: Example node file

## Writing Nodes with rclpy

The `rclpy` library provides the Python client library for ROS 2. Here's a basic node structure:

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        self.get_logger().info('MyRobotNode initialized')

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

## Adding Executables

To make your nodes executable, add them to the `setup.py` file:

```python
entry_points={
    'console_scripts': [
        'my_robot_node = my_robot_package.my_node:main',
    ],
},
```

## Managing Dependencies

Dependencies are specified in both `package.xml` and `setup.py`:

In `package.xml`:
```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
```

In `setup.py`:
```python
install_requires=['setuptools', 'rclpy', 'std_msgs'],
```

## Testing and Linting

ROS 2 packages should include tests. You can add tests in the `test/` directory and configure them in `setup.py`:

```python
tests_require=['pytest'],
```

## Best Practices

- Use descriptive package names that reflect their purpose
- Follow the ROS 2 style guide for Python code
- Include proper documentation and examples
- Use semantic versioning for your packages
- Include a README file explaining the package's purpose and usage

In the next section, we'll explore launch files and how to manage complex ROS 2 systems.
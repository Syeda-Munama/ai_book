# Launch Files

## Introduction to Launch Files

Launch files in ROS 2 allow you to start multiple nodes with a single command. They provide a way to configure and manage complex robot systems by defining which nodes to run, their parameters, and how they should be organized.

## Launch File Structure

Launch files can be written in Python or XML. Python launch files offer more flexibility and are the recommended approach:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='my_robot_node',
            name='my_robot_node',
            parameters=[
                {'param_name': 'param_value'},
            ],
            remappings=[
                ('original_topic', 'new_topic'),
            ],
            output='screen',
        ),
    ])
```

## Launch Arguments

Launch files can accept arguments to make them more flexible:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        Node(
            package='my_robot_package',
            executable='my_robot_node',
            name='my_robot_node',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
```

## Launching Multiple Nodes

Launch files can start multiple nodes with different configurations:

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='sensor_node',
            name='lidar_sensor',
            parameters=[{'sensor_type': 'lidar'}],
        ),
        Node(
            package='my_robot_package',
            executable='controller_node',
            name='motion_controller',
        ),
        Node(
            package='my_robot_package',
            executable='navigation_node',
            name='navigator',
        ),
    ])
```

## Conditions and Logic

Launch files support conditional logic for more complex scenarios:

```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_gui = LaunchConfiguration('use_gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Whether to launch GUI nodes'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_gui),
        ),
    ])
```

## Including Other Launch Files

You can include other launch files to build complex systems:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('other_package'),
                '/launch/other_launch_file.py'
            ])
        ),
    ])
```

## Best Practices

- Organize launch files in a `launch/` directory within your package
- Use descriptive names for launch files
- Use launch arguments to make launch files configurable
- Group related nodes in the same launch file
- Include comments to explain complex launch file logic
- Test launch files to ensure all required nodes start correctly

Launch files are essential for managing complex humanoid robot systems with multiple nodes working together.
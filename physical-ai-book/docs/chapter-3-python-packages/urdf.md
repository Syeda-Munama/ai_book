# URDF for Humanoids

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including links, joints, and their relationships. For humanoid robots, URDF is essential for simulation, visualization, and control.

## URDF Structure

A basic URDF file consists of links and joints:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- A simple joint -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

## Links

Links represent rigid bodies in the robot. Each link has:

- **Visual**: How the link appears in visualization
- **Collision**: How the link interacts in physics simulation
- **Inertial**: Physical properties for dynamics simulation

## Joints

Joints define the relationship between links. Common joint types:

- `revolute`: Rotational joint with limits
- `continuous`: Rotational joint without limits
- `prismatic`: Linear sliding joint with limits
- `fixed`: No movement between links
- `floating`: 6 DOF movement
- `planar`: Movement in a plane

## Humanoid Robot Example

Here's a simplified humanoid upper body:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_upper_body">
  <!-- Torso -->
  <link name="torso">
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
    <parent link="torso"/>
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

  <!-- Left Shoulder -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
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
</robot>
```

## Xacro for Complex Models

For complex humanoid robots, Xacro (XML Macros) simplifies URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Define a macro for creating arms -->
  <xacro:macro name="simple_arm" params="prefix reflect:=1">
    <link name="${prefix}_upper_arm">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
        <origin xyz="0 0 0.15" rpy="${M_PI/2} 0 0"/>
      </visual>
    </link>

    <joint name="${prefix}_shoulder_joint" type="revolute">
      <parent link="torso"/>
      <child link="${prefix}_upper_arm"/>
      <origin xyz="${0.2 * reflect} 0 0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-1.57 * reflect}" upper="${1.57 * reflect}" effort="100" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Torso link -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
  </link>

  <!-- Use the macro to create both arms -->
  <xacro:simple_arm prefix="left" reflect="1"/>
  <xacro:simple_arm prefix="right" reflect="-1"/>

</robot>
```

## Working with URDF in ROS 2

To use URDF in ROS 2, you typically need to:

1. Load the robot description parameter:
```bash
ros2 param set /robot_state_publisher robot_description --string "`cat robot.urdf`"
```

2. Use the robot_state_publisher to publish transforms:
```xml
<Node
  package="robot_state_publisher"
  executable="robot_state_publisher"
  name="robot_state_publisher">
  <param name="robot_description" value="$(find my_robot_description)/urdf/robot.urdf"/>
</Node>
```

## Best Practices

- Start with simple models and gradually add complexity
- Use consistent naming conventions
- Include proper inertial properties for simulation
- Use Xacro for complex robots to avoid duplication
- Test URDF files with tools like `check_urdf`
- Organize URDF files in a dedicated package
- Include both visual and collision geometry
- Use appropriate joint limits for safety

URDF is fundamental for humanoid robot simulation, visualization, and control in ROS 2.
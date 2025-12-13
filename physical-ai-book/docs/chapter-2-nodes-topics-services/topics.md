# Topics and Message Passing

## Understanding Topics

Topics in ROS 2 enable asynchronous communication between nodes through a publish-subscribe pattern. This communication model is particularly useful for humanoid robotics where sensor data needs to be distributed to multiple processing nodes simultaneously.

## Publisher-Subscriber Pattern

The publisher-subscriber pattern works as follows:
- **Publishers** send messages to a topic without knowing who will receive them
- **Subscribers** receive messages from a topic without knowing who sent them
- The ROS 2 middleware handles message routing between publishers and subscribers

### Creating a Publisher in Python

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(String, 'joint_states', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Joint positions: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
```

### Creating a Subscriber in Python

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            String,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint states: {msg.data}')
```

## Message Types

ROS 2 provides a rich set of standard message types for common robot data:
- **std_msgs**: Basic data types (Int32, Float64, String, etc.)
- **sensor_msgs**: Sensor data (JointState, LaserScan, Image, etc.)
- **geometry_msgs**: Geometric primitives (Point, Pose, Twist, etc.)
- **nav_msgs**: Navigation-related messages (Odometry, Path, etc.)

## Quality of Service (QoS) for Topics

QoS settings allow you to control how messages are delivered:
- **Reliability**: Whether messages should be reliably delivered
- **Durability**: Whether late-joining subscribers should receive old messages
- **History**: How many messages to keep in the queue
- **Depth**: Size of the message queue

### Example QoS Configuration

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Configure QoS for real-time sensor data
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)
```

## Topic Communication in Humanoid Robots

For humanoid robots, common topics include:
- Joint states and commands
- Sensor data (IMU, force/torque sensors, cameras)
- Robot state information
- Planning and control commands

## Best Practices

- Use meaningful topic names that reflect the data being transmitted
- Choose appropriate QoS settings based on the data's requirements
- Consider bandwidth and real-time constraints when designing topic architecture
- Use standard message types when possible to promote interoperability

In the next section, we'll cover services for synchronous communication.
# Services

## Introduction to Services

Services in ROS 2 provide synchronous request-response communication between nodes. Unlike topics which enable asynchronous communication, services follow a client-server model where a client sends a request and waits for a response. This pattern is useful for humanoid robotics when you need to perform specific actions and receive immediate results.

## Service Architecture

In the service model:
- **Service Server**: Provides a specific service and responds to requests
- **Service Client**: Sends requests to the service and waits for responses
- The ROS 2 middleware handles request routing and response delivery

### Creating a Service Server in Python

First, define the service interface in a .srv file:

```
# In JointTrajectory.srv
float64[] joint_positions
float64[] joint_velocities
---
bool success
string message
```

Then implement the service server:

```python
import rclpy
from rclpy.node import Node
from your_package.srv import JointTrajectory  # This would be generated from the .srv file

class JointTrajectoryServer(Node):
    def __init__(self):
        super().__init__('joint_trajectory_server')
        self.srv = self.create_service(
            JointTrajectory,
            'execute_joint_trajectory',
            self.execute_trajectory_callback
        )

    def execute_trajectory_callback(self, request, response):
        self.get_logger().info(f'Executing trajectory for joints: {request.joint_positions}')

        # Execute the trajectory (simplified)
        # In a real implementation, this would control the actual joints
        success = True
        message = "Trajectory executed successfully"

        response.success = success
        response.message = message
        return response
```

### Creating a Service Client in Python

```python
import rclpy
from rclpy.node import Node
from your_package.srv import JointTrajectory

class JointTrajectoryClient(Node):
    def __init__(self):
        super().__init__('joint_trajectory_client')
        self.cli = self.create_client(JointTrajectory, 'execute_joint_trajectory')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = JointTrajectory.Request()

    def send_request(self, positions, velocities):
        self.req.joint_positions = positions
        self.req.joint_velocities = velocities

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()
```

## When to Use Services

Services are appropriate when:
- You need synchronous communication
- The operation has a clear request-response pattern
- You need to know the result of an operation before proceeding
- The operation is relatively quick (services are not ideal for long-running operations)

## Service vs. Topic Comparison

| Aspect | Topics | Services |
|--------|--------|----------|
| Communication | Asynchronous | Synchronous |
| Pattern | Publish-Subscribe | Request-Response |
| Timing | Continuous/Periodic | On-demand |
| Use Case | Sensor data, state updates | Actions, queries |

## Service Implementation in Humanoid Robots

Common service use cases in humanoid robots:
- Joint calibration
- Robot state queries
- Emergency stop commands
- Configuration updates
- Trajectory execution requests

## Error Handling in Services

Service implementations should handle errors gracefully:
- Validate input parameters
- Return appropriate success/failure status
- Provide meaningful error messages
- Implement timeouts to prevent hanging clients

In the next section, we'll explore actions, which are designed for long-running operations.
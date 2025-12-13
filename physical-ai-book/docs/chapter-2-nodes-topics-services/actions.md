# Actions

## Introduction to Actions

Actions in ROS 2 are designed for long-running tasks that provide feedback during execution and can be preempted before completion. They combine features of both topics (feedback) and services (goals and results), making them ideal for humanoid robot tasks like navigation, manipulation, and complex motion planning.

## Action Architecture

Actions have three main components:
- **Goal**: The desired outcome sent by the client
- **Feedback**: Continuous updates on the progress of the action
- **Result**: The final outcome of the action when it completes

### Creating an Action Server in Python

First, define the action interface in a .action file:

```
# In HumanoidWalk.action
# Goal: Desired walking parameters
float64 step_size
float64 step_frequency
int32 steps_count

---
# Result: Outcome of the walking action
bool success
string message
float64 actual_distance

---
# Feedback: Continuous updates during walking
float64 current_distance
float64 current_step
int32 steps_completed
string status
```

Then implement the action server:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from your_package.action import HumanoidWalk  # Generated from .action file

class HumanoidWalkActionServer(Node):
    def __init__(self):
        super().__init__('humanoid_walk_action_server')
        self._action_server = ActionServer(
            self,
            HumanoidWalk,
            'humanoid_walk',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received walk goal request')

        # Execute the walking action
        feedback_msg = HumanoidWalk.Feedback()
        result = HumanoidWalk.Result()

        steps_completed = 0
        current_distance = 0.0

        for step in range(goal_handle.request.steps_count):
            # Check if the action has been canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Walk action canceled'
                return result

            # Simulate walking step
            current_distance += goal_handle.request.step_size
            steps_completed += 1

            # Publish feedback
            feedback_msg.current_distance = current_distance
            feedback_msg.current_step = step + 1
            feedback_msg.steps_completed = steps_completed
            feedback_msg.status = f'Walking step {step + 1}/{goal_handle.request.steps_count}'

            goal_handle.publish_feedback(feedback_msg)

            # Sleep to simulate time for each step
            time.sleep(1.0 / goal_handle.request.step_frequency)

        # Action completed successfully
        goal_handle.succeed()
        result.success = True
        result.message = 'Walking completed successfully'
        result.actual_distance = current_distance

        return result
```

### Creating an Action Client in Python

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from your_package.action import HumanoidWalk

class HumanoidWalkActionClient(Node):
    def __init__(self):
        super().__init__('humanoid_walk_action_client')
        self._action_client = ActionClient(
            self,
            HumanoidWalk,
            'humanoid_walk'
        )

    def send_goal(self, step_size, step_frequency, steps_count):
        goal_msg = HumanoidWalk.Goal()
        goal_msg.step_size = step_size
        goal_msg.step_frequency = step_frequency
        goal_msg.steps_count = steps_count

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: {feedback.status} - Distance: {feedback.current_distance:.2f}m'
        )

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.message} - Distance: {result.actual_distance:.2f}m')
```

## When to Use Actions

Actions are appropriate when:
- The operation takes a significant amount of time
- You need to provide feedback during execution
- The operation might need to be canceled or preempted
- You need both the final result and progress information

## Action vs. Service vs. Topic Comparison

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| Communication | Async | Sync | Async |
| Feedback | Continuous | No | Continuous |
| Cancellation | N/A | No | Yes |
| Use Case | Streaming data | Quick queries | Long operations |

## Action Implementation in Humanoid Robots

Common action use cases in humanoid robots:
- Walking and navigation
- Manipulation tasks
- Complex motion planning
- Object grasping and placement
- Humanoid dance or gesture sequences

## Best Practices for Actions

- Design goals to be achievable and well-defined
- Provide meaningful feedback at regular intervals
- Implement proper cancellation handling
- Use appropriate result messages that capture the outcome
- Consider the timing and frequency of feedback updates

## Diagram References

<!-- ![Topic Communication Flow](/img/topic-communication-flow.png) -->

*Figure 1: Communication flow in the publish-subscribe pattern used by topics.*

<!-- ![Service Communication](/img/service-communication.png) -->

*Figure 2: Request-response pattern used by services.*

In the next section, we'll work on an exercise to implement a publisher/subscriber system for humanoid joint control.
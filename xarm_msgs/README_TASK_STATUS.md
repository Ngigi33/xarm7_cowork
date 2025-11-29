/task_status — Topic README

Purpose

The `/task_status` ROS2 topic is used to publish status updates for tasks executed by the sequence runner(s). Each message provides the task filename, current state, optional progress, a timestamp, and a task id for correlating messages.

Message definition

File: `src/xarm_msgs/msg/TaskStatus.msg`

```
string task_name      # e.g., pinion_gear_task.py
string state          # Starting, In-progress, Finished, Error
builtin_interfaces/Time timestamp
float32 progress      # Optional: 0.0 - 100.0
int32 task_id         # optional ID for tracking tasks
```

Field explanation

- `task_name` (string): the filename or logical name of the task. Example: `pinion_gear_task.py`.
- `state` (string): the current state. Use a small stable vocabulary; the sequence runner uses: `Starting`, `In-progress`, `Finished`, `Error`.
- `timestamp` (builtin_interfaces/Time): ROS2 time when the status was published. Useful for ordering and latency measurements.
- `progress` (float32): optional progress percentage (0.0 — 100.0). When not applicable, publishers may send 0.0 or omit updates.
- `task_id` (int32): optional identifier assigned by the publisher to correlate messages belonging to the same task (useful when tasks run concurrently or for logs).

Publisher

- The C++ node `task_sequencer.cpp` publishes `TaskStatus` messages to the `task_status` topic before, during, and after running each Python script.
- Typical publication pattern used in `task_sequencer.cpp`:
  - Publish `Starting` with progress 0.0 and a new `task_id`.
  - Publish `In-progress` with intermediate progress (e.g. 50.0) during execution.
  - Publish `Finished` with progress 100.0 after success.
  - Publish `Error` if the script returned a non-zero exit code.

Example commands (quick)

- Echo messages on the topic (raw):

```bash
ros2 topic echo /task_status
```

- Echo message fields, e.g. only task_name and state:

```bash
ros2 topic echo /task_status --field task_name --field state
```

- List topic types to confirm the custom message is available:

```bash
ros2 topic list -t | grep task_status -A1
```

Python subscriber example

This minimal example subscribes to `/task_status` and logs messages.

```python
# simple_task_status_sub.py
import rclpy
from rclpy.node import Node
from xarm_msgs.msg import TaskStatus

class TaskStatusSubscriber(Node):
    def __init__(self):
        super().__init__('task_status_sub')
        self.sub = self.create_subscription(TaskStatus, 'task_status', self.cb, 10)

    def cb(self, msg: TaskStatus):
        self.get_logger().info(f"Task {msg.task_name} | State: {msg.state} | Progress: {msg.progress} | id={msg.task_id}")

if __name__ == '__main__':
    rclpy.init()
    node = TaskStatusSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

C++ subscriber example snippet

```cpp
// Create a subscription in a rclcpp::Node-derived class
status_sub_ = this->create_subscription<xarm_msgs::msg::TaskStatus>(
    "task_status", 10,
    [this](const xarm_msgs::msg::TaskStatus::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Task %s state=%s progress=%.1f id=%d",
                    msg->task_name.c_str(), msg->state.c_str(), msg->progress, msg->task_id);
    });
```

Best practices

- Use small, stable state values (Starting, In-progress, Finished, Error). Avoid free-form text for machine parsing.
- Publish timestamps using `this->now()` (C++) or `self.get_clock().now()` (rclpy) so consumers can compute latencies.
- Prefer incremental progress updates only when meaningful. Excessive frequent updates may add unnecessary network load.
- Use `task_id` to correlate logs and to deduplicate messages when multiple instances of the same script run.

Troubleshooting

- `ros2 topic echo /task_status` shows nothing:
  - Ensure the `xarm_msgs` package is built and sourced (`colcon build` + `source install/setup.bash`).
  - Confirm the publisher node is running (e.g., `ros2 node list`).
  - Use `ros2 topic list` to verify topic name.

- Message type not found:
  - Make sure the `TaskStatus.msg` is declared in `xarm_msgs` package and that package built successfully.
  - Re-source your workspace (`source install/setup.bash`) after building.

- Messages show odd timestamps or zero progress: check that the publisher sets timestamp and progress intentionally; the example publisher sets simple simulated progress.



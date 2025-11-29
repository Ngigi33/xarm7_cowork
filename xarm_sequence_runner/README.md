# xarm_sequence_runner

This package provides a small C++ node (`task_sequencer.cpp`) that triggers two ordered sequences of Python task scripts and exposes two ROS2 services to control them.

Quick commands (copy/paste)

```bash
# Build the package (from your workspace root):
colcon build --packages-select xarm_sequence_runner
source install/setup.bash

# Run the C++ node:
ros2 run xarm_sequence_runner task_sequencer

# Start the main sequence (call with custom Trigger.srv):
ros2 service call /start_task_sequence xarm_msgs/srv/Trigger "{run: true}"

# Start the post-manual (auto) sequence:
ros2 service call /start_auto_sequence xarm_msgs/srv/Trigger "{run: true}"

#Open gripper
ros2 service call /start_open_gripper xarm_msgs/srv/Trigger "{run: true}"

```

Overview
- `start_task_sequence` service: starts the main sequence of tasks (runs a list of Python scripts in order). After the main sequence completes, the node automatically starts the post-manual sequence in a background thread and waits for an explicit "auto" trigger if needed.
- `start_auto_sequence` service: starts the post-manual (auto) sequence directly.
- Custom service type: `xarm_msgs/srv/Trigger` is used by both services. It includes a `bool run` request field and returns `bool success` and `string message` in the response.

Files
- `src/task_sequencer.cpp` — the C++ node implementing the two services and the script lists. Update the vectors in this file to change which Python scripts are executed and their order.
- `../xarm_msgs/srv/Trigger.srv` — custom service definition (bool run --- bool success, string message).

How it works
1. The node defines two vectors of shell commands that run Python task scripts under your project venv (via `bash -c "cd ... && source .../xarm_env/bin/activate && python <script>"`).
2. When the `start_task_sequence` service is called with `run: true`, the node executes each command in `main_sequence_scripts_` sequentially using `system()`.
3. After the main sequence completes, the node automatically triggers the post-manual sequence in a detached thread (so the main service call can return quickly). The post-manual sequence is a separate list (`post_manual_sequence_scripts_`).
4. Alternatively, call the `start_auto_sequence` service to run only the post-manual sequence on demand.

Custom service: Trigger.srv
```
# Request: set `run` to true to trigger the service action. If omitted
# (old clients), the service will behave as before and start the action.
bool run
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error message
```

Example usage (ROS2)
- Start the main sequence:

```
ros2 service call /start_task_sequence xarm_msgs/srv/Trigger "{run: true}"
```

- Start the post-manual (auto) sequence:

```
ros2 service call /start_auto_sequence xarm_msgs/srv/Trigger "{run: true}"
```

Notes & tips
- The C++ node uses `system()` to run scripts via shell commands. The example `task_sequencer.cpp` sources the Python virtualenv activation script before running tasks so the Python environment (packages, SDK) is correctly available. Make sure the `venv_activate` path is correct for your workspace.
- Adjust the `base_path`, `tasks_path` and `venv_activate` variables in `task_sequencer.cpp` to match your environment. In the provided file the base path is set to `/home/vmlabs/xarm7_cowork/dev_ws/src` and `tasks` are in `.../tasks`.
- The node logs progress with `RCLCPP_INFO` and returns non-zero system command results as errors in the service response.
- If a post-manual sequence includes a script that itself triggers a manual mode, you will need to handle coordination so it doesn't block unexpectedly. The C++ example simply runs the post-manual scripts sequentially.

Building and running
1. From your ROS2 workspace root (where `src/` lives):

```bash
colcon build --packages-select xarm_sequence_runner
source install/setup.bash
```

2. Run the node:

```bash
ros2 run xarm_sequence_runner task_sequencer
```

3. Call the services as shown above to start sequences.

Troubleshooting
- Service not found: ensure the package was built and sourced (`source install/setup.bash`) and the node is running.
- Python scripts fail: verify the `venv_activate` path, that the venv has the required packages installed, and that the working `tasks` directory contains the scripts.
- Permission issues: ensure scripts are readable and the user has permission to execute `bash -c` commands.

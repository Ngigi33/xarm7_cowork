# xarm7_cowork
This workspace contains helper scripts, a Python task sequencer, and a small C++ ROS2 node
that run ordered sequences of Python task scripts for the xArm test setup.

Quick overview
- `dev_ws/src/run_task_sequencer.py` — Python sequencer (interactive + optional ROS2 Trigger service) to run task scripts in `tasks/`.
- `dev_ws/src/xarm_sequence_runner/src/task_sequencer.cpp` — C++ node exposing ROS2 services to start the main and post-manual sequences.
- `dev_ws/src/run_xarm7.py` — helper that opens GNOME Terminal tabs and runs configured commands; press `q` in the launcher terminal to stop and close tabs.
- `dev_ws/src/tasks/` — directory with task scripts (Python files) that perform robot operations (examples live in the repo's `tasks` folder).
- `dev_ws/src/xarm_custom_interfaces/` — custom ROS2 interfaces (includes `Trigger.srv` and `TaskStatus.msg`).

Quick setup (local developer machine)
1. Create and activate a Python virtualenv (optional but recommended):

```bash
python3 -m venv xarm_env
source xarm_env/bin/activate
```

2. Install project Python dependencies (if any) and SDK in the venv (example):

```bash
cd xArm-Python-SDK
pip install .
```

3. Build ROS2 packages (from workspace root, where `src/` lives):

```bash
colcon build
source install/setup.bash
```

Run the sequencers and services
- Run the Python sequencer interactively:

```bash
python dev_ws/src/run_task_sequencer.py
# In the prompt: type `Start` to run DEFAULT_SEQUENCE. After a manual step type `auto` to resume.
```

- Run the C++ sequence runner node (built with colcon):

```bash
ros2 run xarm_sequence_runner task_sequencer
```

- Control the C++ node with the custom Trigger service:

```bash
# start main sequence
ros2 service call /start_main_sequence xarm_custom_interfaces/srv/Trigger "{run: true}"
# start post-manual (auto) sequence
ros2 service call /start_auto_sequence xarm_custom_interfaces/srv/Trigger "{run: true}"
```

Topic and messages
- `/task_status` publishes `xarm_custom_interfaces/msg/TaskStatus` messages from the C++ sequencer. Use `ros2 topic echo /task_status` to monitor task names, states and progress.

Configuration notes
- Edit `dev_ws/src/run_task_sequencer.py` to configure `DEFAULT_SEQUENCE` and `DEFAULT_POST_MANUAL_SEQUENCE` (ordered lists of task filenames).
- Edit `dev_ws/src/xarm_sequence_runner/src/task_sequencer.cpp` to change the shell commands and venv activation path that run the Python scripts.
- Update `./example/wrapper/robot.conf` to set the robot controller IP when you deploy to hardware.

Troubleshooting
- If a service or message type is missing: re-build the workspace and source it:

```bash
colcon build --packages-select xarm_custom_interfaces xarm_sequence_runner
source install/setup.bash
```

- If tasks fail to run from the C++ node, verify the `venv_activate` path and that the venv has required packages installed.

Where to get help or extend
- The Python sequencer supports interactive `Start`/`auto` flow and an optional ROS2 Trigger-based interface. See `run_task_sequencer.py` for usage and CLI flags.
- The C++ node publishes `TaskStatus` messages to `/task_status`; see `xarm_custom_interfaces/msg/TaskStatus.msg` for the schema and `xarm_custom_interfaces/README_TASK_STATUS.md` for usage examples.


# xarm7_cowork

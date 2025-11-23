run_xarm7.py — quick README

What this script does

- `run_xarm7.py` is a small helper script that opens one or more new GNOME Terminal tabs and runs configured commands (typically ROS2 nodes or helper commands) in each tab.
- It provides an in-terminal keyboard control in the original terminal: press `q` to kill the launched processes and close the opened tabs.

Where it lives

- Script path: `dev_ws/src/run_xarm7.py`

Why use it

- Helpful for starting the sequence runner node(s) quickly in separate terminal tabs and having a single local keyboard control to stop them.
- Keeps logs visible in separate terminal tabs while allowing a single control terminal to manage lifecycle.

Prerequisites

- GNOME Terminal (the script uses the `gnome-terminal` command and `--tab` option).
- Python 3 and normal user permissions to spawn terminals/processes.
- If launching ROS2 nodes, a sourced workspace and a working ROS2 installation are needed in the environment where the commands run.

How the script is configured

- The file contains a `workspace_folder` variable and a `commands` list near the top. Each entry in `commands` is a shell command that will be executed inside a separate new terminal tab.
- Example (provided in the repo):

  ```python
  workspace_folder = "/home/vmlabs/xarm7_cowork/dev_ws"
  commands = [
      f"cd {workspace_folder}; source install/setup.bash; ros2 run xarm_sequence_runner task_sequencer",
  ]
  ```

How to run

1. Make sure your workspace is built and sourced if the commands depend on ROS2 packages:

```bash
# from workspace root (one level above src/)
colcon build
source install/setup.bash
```

2. Run the helper script from any terminal:

```bash
python dev_ws/src/run_xarm7.py
```

3. The script will open the configured commands in new GNOME Terminal tabs. Keep the original terminal open to control the processes.

Keyboard controls

- Press `q` in the original controlling terminal (where you ran `run_xarm7.py`) to stop all launched processes and close (or cause to close) the opened tabs.
- The script performs a process search for the command strings and sends SIGKILL to matching PIDs to terminate them.

Notes and behavior details

- The script opens each command in a new GNOME Terminal tab using `subprocess.Popen(["gnome-terminal", "--tab", "--", "bash", "-c", "<command>"])`.
- When you press `q`, the script tries to find running processes that match the command tail and kills them with SIGKILL. This ensures the launched processes terminate and their terminal tabs will typically close.
- Because matching processes by command text can be imprecise, make sure the command strings in `commands` are specific enough to identify the intended processes.

Tips and customization

- Add more commands to the `commands` list to open multiple tabs for other nodes/tools.
- If you want to run terminals with different profiles, titles, or keep terminals open after the command ends, adjust the `gnome-terminal` arguments.
- On systems without GNOME Terminal or where `gnome-terminal --tab` behaves differently, adapt the script to use `xterm`, `konsole`, or a different terminal emulator.

Troubleshooting

- Tabs do not open: ensure `gnome-terminal` is installed and available on PATH.
- Processes are not killed on `q`: the `kill_processes_by_name` helper looks for process lines containing the command; if the matching fails, run `ps aux | grep <command>` to debug. You can also modify the script to store Popen objects and kill their process groups directly.
- Permission denied errors: ensure the user can spawn terminals and the commands do not require elevated privileges.



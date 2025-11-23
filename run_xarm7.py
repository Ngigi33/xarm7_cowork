import subprocess
import os
import signal
import sys
import termios
import tty
import time

# List of commands to run in separate tabs
workspace_folder = "/home/vmlabs/xarm7_cowork/dev_ws"
commands = [
    f"cd {workspace_folder}; source install/setup.bash; ros2 run xarm_sequence_runner task_sequencer",
    # Add more commands if needed
]

LOG_CSV_FILE = os.path.join(workspace_folder, "log.csv")

# Store opened processes
processes = []

def run_in_new_tab(command):
    """
    Opens a new terminal tab and runs the specified command.
    Returns the Popen object.
    """
    proc = subprocess.Popen([
        "gnome-terminal",
        # "--disable-factory",  # ensure independent tab
        "--tab",
        "--",
        "bash",
        "-c",
        f"{command}"
    ])
    return proc

def kill_processes_by_name(process_name):
    """
    Kills all processes matching the given command name.
    """
    ps_command = f"ps aux | grep '{process_name}' | grep -v grep"
    try:
        output = subprocess.check_output(ps_command, shell=True, text=True)
        pids = [int(line.split()[1]) for line in output.splitlines()]
        for pid in pids:
            print(f"Killing PID {pid} -> {process_name}")
            os.kill(pid, signal.SIGKILL)
    except subprocess.CalledProcessError:
        pass

def get_char():
    """Get a single character from stdin without Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    # Start all commands in new tabs
    for cmd in commands:
        proc = run_in_new_tab(cmd)
        processes.append(proc)
        time.sleep(0.5)  # small delay to stagger tab opening

    # print("\nCommands running in new tabs.")
    # print("Press 'q' to kill all processes and close tabs.")
    print("\n" + "="*50)
    print("   KEYBOARD CONTROLS:")
    print("   Press 'q' to QUIT the program")
    print("="*50)
        

    # Wait for user keypress
    while True:
        char = get_char().lower()
        if char == 'q':
            print("\nKilling all processes...")
            for cmd in commands:
                kill_processes_by_name(cmd.split(";")[-1].strip())
            print("Done. Exiting.")
            break

if __name__ == "__main__":
    main()

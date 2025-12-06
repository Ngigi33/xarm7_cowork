import subprocess
import os
import signal
import sys
import termios
import tty
import time
import socket

# List of commands to run in separate tabs
workspace_folder = "/home/vmlabs/xarm7_cowork/dev_ws"
commands = [
    f"cd {workspace_folder}; source install/setup.bash; ros2 run xarm_sequence_runner task_sequencer",
    f"cd {workspace_folder}; source install/setup.bash; ros2 run xarm_py_nodes joint_info_publisher",
    f"cd {workspace_folder}; source install/setup.bash; ros2 run xarm_sequence_runner emergency_stop_subscriber",
    # f"cd {workspace_folder}; source install/setup.bash; ros2 run xarm_sequence_runner process_launcher",
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

# def get_char():
#     """Get a single character from stdin without Enter"""
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     try:
#         tty.setraw(fd)
#         ch = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return ch
def get_char():
    if not sys.stdin.isatty():
        # Fallback when run from GUI or pipe
        return input("\nEnter command (q to quit): ").strip()[0]

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        char = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return char


def get_ip_address_simple():
    """
    Lists all available network interfaces and their IP addresses,
    then allows the user to select which one to use.
    """
    import netifaces
    
    try:
        print("\n" + "="*50)
        print("🌐 AVAILABLE NETWORK INTERFACES:")
        print("="*50)
        
        interfaces = netifaces.interfaces()
        valid_interfaces = []
        
        for i, interface in enumerate(interfaces):
            try:
                # Get IPv4 addresses for this interface
                addrs = netifaces.ifaddresses(interface)
                if netifaces.AF_INET in addrs:
                    ipv4_info = addrs[netifaces.AF_INET][0]
                    ip_addr = ipv4_info['addr']
                    netmask = ipv4_info.get('netmask', 'N/A')
                    
                    # Skip loopback addresses
                    if not ip_addr.startswith('127.'):
                        valid_interfaces.append((interface, ip_addr, netmask))
                        print(f"{len(valid_interfaces)}. {interface:12} - {ip_addr:15} (Netmask: {netmask})")
                        
            except (KeyError, IndexError):
                # Skip interfaces without IPv4 addresses
                continue
        
        if not valid_interfaces:
            print("No valid network interfaces found!")
            # Fallback to original method
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        
        print("="*50)
        
        # Let user choose interface
        while True:
            try:
                choice = input(f"Select interface (1-{len(valid_interfaces)}) or press Enter for auto-detect: ").strip()
                
                if choice == "":
                    # Auto-detect using original method
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.connect(("8.8.8.8", 80))
                    ip = s.getsockname()[0]
                    s.close()
                    print(f"Auto-detected IP: {ip}")
                    return ip
                
                choice_num = int(choice)
                if 1 <= choice_num <= len(valid_interfaces):
                    selected_interface, selected_ip, selected_netmask = valid_interfaces[choice_num - 1]
                    print(f"Selected: {selected_interface} - {selected_ip}")
                    return selected_ip
                else:
                    print(f"Invalid choice. Please enter a number between 1 and {len(valid_interfaces)}")
                    
            except ValueError:
                print("Invalid input. Please enter a number or press Enter for auto-detect.")
            except KeyboardInterrupt:
                print("\nOperation cancelled by user")
                return None
                
    except ImportError:
        print("netifaces module not found. Using fallback method...")
        print("Install with: pip install netifaces")
        # Fallback to original method
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception as e:
            print(f"Error getting IP address: {e}")
            return None
    except Exception as e:
        print(f"Error listing network interfaces: {e}")
        # Fallback to original method
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception as fallback_e:
            print(f"Error getting IP address: {fallback_e}")
            return None




def main():
    # Start all commands in new tabs
    ip_choice = input("Do you want to connect to Unity? (y/n) ").strip().lower()

    if ip_choice == "y":
        ros_ip = str(get_ip_address_simple())




    if ip_choice == "y":
        commands.append(
            f"cd {workspace_folder}; source install/setup.bash; "
            f"ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:={ros_ip}"
        )






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

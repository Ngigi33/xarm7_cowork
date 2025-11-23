import time
import csv
from datetime import datetime

# ---------------------------
# CONFIG
# ---------------------------
log_file_path = "/tmp/task_terminal.log"  # path to the terminal output you want to monitor (or pipe output to this file)
csv_file = "/home/vmlabs/task_log_monitor.csv"
sequence_id = "Main_001"

# ---------------------------
# HELPER FUNCTIONS
# ---------------------------
def get_current_time():
    return datetime.now()

def get_time_str(dt):
    return dt.strftime("%Y-%m-%d %H:%M:%S")

# ---------------------------
# MAIN SCRIPT
# ---------------------------
def main():
    # Dictionary to store task start times
    task_start_times = {}
    last_task_end = None

    # Open CSV and write header if file does not exist
    try:
        with open(csv_file, 'x', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Task Name", "Sequence ID", "Start Time", "End Time", "Duration(s)", "Idle Time(s)"])
    except FileExistsError:
        pass  # file already exists

    # Open the terminal log file and follow it
    with open(log_file_path, 'r') as f:
        # Seek to the end if you want to monitor only new output
        f.seek(0, 2)

        while True:
            line = f.readline()
            if not line:
                time.sleep(0.1)  # wait for new lines
                continue

            line = line.strip()
            if not line:
                continue

            print(line)  # print monitored line

            # Detect task Started
            if "Started" in line:
                task_name = extract_task_name(line)
                task_start_times[task_name] = get_current_time()

            # Detect task Finished
            elif "Finished" in line:
                task_name = extract_task_name(line)
                start_time = task_start_times.get(task_name, None)
                end_time = get_current_time()
                if start_time:
                    duration = (end_time - start_time).total_seconds()
                    idle_time = (start_time - last_task_end).total_seconds() if last_task_end else 0
                    last_task_end = end_time

                    # Append to CSV
                    with open(csv_file, 'a', newline='') as csv_f:
                        writer = csv.writer(csv_f)
                        writer.writerow([
                            task_name,
                            sequence_id,
                            get_time_str(start_time),
                            get_time_str(end_time),
                            f"{duration:.2f}",
                            f"{idle_time:.2f}"
                        ])

                    # Remove start time to avoid duplicate
                    del task_start_times[task_name]

# ---------------------------
# FUNCTION TO EXTRACT TASK NAME
# ---------------------------
def extract_task_name(line):
    """
    Example: "Started" or "[148] Started" 
    You may customize based on your terminal format.
    If task name is not printed, you can assign a default name or sequence order.
    """
    # For simplicity, use the line number hash as task name
    return f"Task_{hash(line) % 1000}"

# ---------------------------
# ENTRY POINT
# ---------------------------
if __name__ == "__main__":
    main()

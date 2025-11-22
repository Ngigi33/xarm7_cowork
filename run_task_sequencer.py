# #!/usr/bin/env python3
# """
# run_task_sequencer.py

# Simple task sequencer that can be triggered interactively (type "Start") or via
# a ROS2 service call (/run_tasks using std_srvs/Trigger) when rclpy is available.

# Behavior:
# - Scans the sibling `tasks/` directory (relative to repository root: expected
#   to be `~/xarm7_cowork/tasks/`) and finds runnable Python files (*.py).
# - By default runs all task files sorted by name. You can pass specific filenames
#   with `--tasks file1.py,file2.py`.
# - Each task is launched as a subprocess using the same Python executable (sys.executable)
#   and the working directory set to the `tasks/` folder so relative imports work.

# Usage (CLI):
#   python run_task_sequencer.py           # interactive prompt, type Start to run
#   python run_task_sequencer.py --start  # immediately runs sequence
#   python run_task_sequencer.py --tasks pinion_gear_task.py

# Usage (ROS2):
#   If rclpy is installed, the script will expose a `/run_tasks` service (std_srvs/Trigger).
#   Calling this service triggers the same sequence and returns success/message.

# This file is intentionally lightweight and avoids adding ROS2 runtime as a hard dependency.
# """

# from __future__ import annotations

# import argparse
# import logging
# import os
# import shlex
# import subprocess
# import sys
# import threading
# import time
# from typing import Iterable, List, Optional

# logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
# logger = logging.getLogger("run_task_sequencer")

# # Edit this list to define the exact sequence/order of tasks to run from code.
# # Example:
# # DEFAULT_SEQUENCE = ["pinion_gear_task.py", "other_task.py"]
# # If left empty, the sequencer will discover tasks in the tasks/ directory.
# DEFAULT_SEQUENCE: List[str] = [
#     "pinion_gear_task.py",
#     "pinion_spindle_task.py",
#     "spur_spindle_task.py",
#     "adhesive_applier_task.py",
#     "manual_mode_changer.py",
#     # "adhesive_returner_task.py",
#     # "cover_plate_task.py",
# ]

# # Optional post-manual sequence: edit this list to run a different set of tasks
# # immediately after the manual step is resumed (via 'auto' or /resume_sequence).
# DEFAULT_POST_MANUAL_SEQUENCE: List[str] = [
#     "adhesive_returner_task.py",
#     "cover_plate_task.py",
#     "manual_mode_changer.py",
# ]

# # Runtime container for the post-manual sequence (populated from CLI or left as DEFAULT)
# POST_MANUAL_SEQUENCE: List[str] = [
#     "adhesive_returner_task.py",
#     "cover_plate_task.py",
#     "manual_mode_changer.py",
# ]

# # Event used to resume the sequence after a manual step.
# resume_event = threading.Event()
# # Track the currently running sequence thread so interactive prompt can signal it.
# _current_sequence_thread: Optional[threading.Thread] = None
# _sequence_thread_lock = threading.Lock()


# def find_tasks(tasks_dir: str) -> List[str]:
#     """Return list of python task filenames (basename) found in tasks_dir sorted."""
#     try:
#         names = [f for f in os.listdir(tasks_dir) if f.endswith(".py")]
#     except FileNotFoundError:
#         logger.error("tasks directory not found: %s", tasks_dir)
#         return []
#     # skip helper/init files
#     names = [n for n in names if not n.startswith("_") and n != "__init__.py"]
#     names.sort()
#     return names


# def run_task_file(python_exe: str, tasks_dir: str, filename: str) -> int:
#     """Run a single task file and stream its output. Returns process returncode."""
#     path = os.path.join(tasks_dir, filename)
#     if not os.path.isfile(path):
#         logger.error("Task not found: %s", path)
#         return 127

#     logger.info("Starting task: %s", filename)
#     logger.info("In-progress : %s", filename)
#     # Use the same python executable that runs this script (so venv is respected)
#     cmd = [python_exe, path]

#     # Start the process and stream output
#     proc = subprocess.Popen(
#         cmd,
#         cwd=tasks_dir,
#         stdout=subprocess.PIPE,
#         stderr=subprocess.PIPE,
#         text=True,
#         bufsize=1,
#         universal_newlines=True,
#     )

#     # Stream stdout and stderr
#     assert proc.stdout is not None and proc.stderr is not None
#     def stream_pipe(pipe, prefix=""):
#         for line in iter(pipe.readline, ""):
#             print(prefix + line.rstrip())
#         pipe.close()

#     t_out = threading.Thread(target=stream_pipe, args=(proc.stdout, ""), daemon=True)
#     t_err = threading.Thread(target=stream_pipe, args=(proc.stderr, "ERR: "), daemon=True)
#     t_out.start()
#     t_err.start()

#     proc.wait()
#     t_out.join()
#     t_err.join()

#     logger.info("Task finished: %s (exit %d)", filename, proc.returncode)
#     return proc.returncode


# def run_sequence(tasks_dir: str, filenames: Optional[Iterable[str]] = None, *, pause_on_manual: bool = True) -> List[int]:
#     """Run a sequence of tasks.

#     If filenames is None, run all tasks found in tasks_dir. If pause_on_manual is False,
#     the sequencer will not pause when encountering filenames containing 'manual'.

#     Returns list of return codes for each run in order.
#     """
#     python_exe = sys.executable
#     if filenames is None:
#         files = find_tasks(tasks_dir)
#     else:
#         files = list(filenames)

#     if not files:
#         logger.warning("No tasks to run in %s", tasks_dir)
#         return []

#     results = []
#     for f in files:
#         code = run_task_file(python_exe, tasks_dir, f)
#         results.append(code)
#         # If a manual step is detected (filename contains 'manual'), pause until resume_event is set
#         try:
#             name_low = f.lower()
#         except Exception:
#             name_low = ""
#         if pause_on_manual and "manual" in name_low:
#             logger.info("Manual mode detected after %s — waiting for 'auto' input or resume service...", f)
#             # clear any previous state and wait indefinitely for resume
#             resume_event.clear()
#             resume_event.wait()
#             logger.info("Resuming sequence after manual step: %s", f)
#             # If a post-manual sequence is configured, run it now and then continue.
#             # Run post-manual without further manual pauses to avoid nested waits.
#             try:
#                 if POST_MANUAL_SEQUENCE:
#                     logger.info("Running post-manual sequence: %s", POST_MANUAL_SEQUENCE)
#                     run_sequence(tasks_dir, POST_MANUAL_SEQUENCE, pause_on_manual=False)
#             except Exception as e:
#                 logger.error("Error running post-manual sequence: %s", e)
#         # Optional: stop on failure
#         if code != 0:
#             logger.warning("Task %s returned non-zero (%d). Continuing to next task.", f, code)
#     return results


# def _interactive_mode(tasks_dir: str, default_tasks: Optional[List[str]] = None):
#     print("Task sequencer interactive mode. Type 'Start' to run tasks, 'list' to show tasks, 'auto' to resume after manual, 'q' to quit.")
#     global _current_sequence_thread
#     while True:
#         try:
#             val = input("> ").strip()
#         except (EOFError, KeyboardInterrupt):
#             print()
#             break

#         if not val:
#             continue
#         if val.lower() in ("q", "quit", "exit"):
#             break
#         if val.lower() == "list":
#             found = find_tasks(tasks_dir)
#             print("Available tasks:")
#             for f in found:
#                 print(" - " + f)
#             continue
#         if val.lower().startswith("run "):
#             # allow: run file1.py,file2.py
#             rest = val[4:].strip()
#             to_run = [item.strip() for item in rest.split(",") if item.strip()]
#             # run in background so the interactive prompt can accept 'auto' to resume
#             with _sequence_thread_lock:
#                 if _current_sequence_thread is None or not _current_sequence_thread.is_alive():
#                     _current_sequence_thread = threading.Thread(target=run_sequence, args=(tasks_dir, to_run), daemon=True)
#                     _current_sequence_thread.start()
#                 else:
#                     print("A sequence is already running. Wait for it to finish or use 'auto' to resume a manual step.")
#             continue
#         if val.lower() == "start" or val.lower() == "startall":
#             # If a default task list was provided, use it. Otherwise, if DEFAULT_SEQUENCE
#             # is configured in the file, use that. If neither, auto-discover.
#             seq = default_tasks if default_tasks is not None else (DEFAULT_SEQUENCE if DEFAULT_SEQUENCE else None)
#             with _sequence_thread_lock:
#                 if _current_sequence_thread is None or not _current_sequence_thread.is_alive():
#                     _current_sequence_thread = threading.Thread(target=run_sequence, args=(tasks_dir, seq), daemon=True)
#                     _current_sequence_thread.start()
#                 else:
#                     print("A sequence is already running. Wait for it to finish or use 'auto' to resume a manual step.")
#             continue
#         if val.lower() == "auto":
#             # signal resume for manual steps
#             if resume_event.is_set():
#                 print("Sequence is not waiting for resume (already set).")
#             else:
#                 resume_event.set()
#                 print("Resuming sequence (auto)")
#             continue
#         print("Unknown command. Use 'Start', 'list', 'run <file1,file2>', or 'q'.")


# def try_setup_ros_service(tasks_dir: str):
#     """Try to set up a ROS2 service /run_tasks that triggers the sequence.

#     Returns the rclpy node if ROS2 is available and setup succeeded, otherwise None.
#     """
#     try:
#         import rclpy
#         from rclpy.node import Node
#         from std_srvs.srv import Trigger
#     except Exception as e:  # ImportError or rclpy not installed
#         logger.debug("ROS2 (rclpy) not available: %s", e)
#         return None

#     class SequencerNode(Node):
#         def __init__(self):
#             super().__init__("run_task_sequencer")
#             self.srv = self.create_service(Trigger, "run_tasks", self.handle_run)
#             # Additional service to resume sequence after a manual step
#             self.resume_srv = self.create_service(Trigger, "resume_sequence", self.handle_resume)
#             self.get_logger().info("/run_tasks service ready")

#         def handle_run(self, request, response):
#             # Trigger the sequence in a background thread to avoid blocking the service
#             self.get_logger().info("/run_tasks called — starting sequence")
#             thread = threading.Thread(target=self._run_background, daemon=True)
#             thread.start()
#             # record the running thread so interactive prompt and resume logic can see it
#             try:
#                 with _sequence_thread_lock:
#                     global _current_sequence_thread
#                     _current_sequence_thread = thread
#             except Exception:
#                 pass
#             response.success = True
#             response.message = "Task sequence started"
#             return response

#         def _run_background(self):
#             results = run_sequence(tasks_dir)
#             self.get_logger().info("Sequence finished: %s", str(results))

#         def handle_resume(self, request, response):
#             self.get_logger().info("/resume_sequence called — setting resume flag")
#             resume_event.set()
#             response.success = True
#             response.message = "Sequence resumed"
#             return response

#     try:
#         rclpy.init()
#         node = SequencerNode()

#         # Spin the node in a background thread so the main thread can continue
#         spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
#         spin_thread.start()
#         logger.info("ROS2 service /run_tasks is available")
#         return node
#     except Exception as e:
#         logger.error("Failed to initialize ROS2 service: %s", e)
#         try:
#             rclpy.shutdown()
#         except Exception:
#             pass
#         return None


# def main():
#     parser = argparse.ArgumentParser(description="Run task sequencer (CLI + optional ROS2 service)")
#     parser.add_argument("--tasks-dir", default=os.path.abspath(os.path.join(os.getcwd(), "../src/tasks")),
#                         help="Path to tasks directory (default: ../src/tasks relative to cwd)")
#     parser.add_argument("--tasks", help="Comma-separated list of task filenames to run (overrides default)")
#     parser.add_argument("--start", action="store_true", help="Start sequence immediately (non-interactive)")
#     parser.add_argument("--no-ros", action="store_true", help="Do not attempt to start ROS2 service even if rclpy installed")
#     parser.add_argument("--use-sequence", action="store_true",
#                         help="Use the hard-coded DEFAULT_SEQUENCE list in this file instead of auto-discovering tasks")
#     parser.add_argument("--sequence-file", help="Path to a file listing task filenames (one per line) to use as sequence")
#     parser.add_argument("--use-post-sequence", action="store_true",
#                         help="Use the DEFAULT_POST_MANUAL_SEQUENCE defined in the file for post-manual steps")
#     parser.add_argument("--post-sequence-file", help="Path to a file listing post-manual task filenames (one per line)")
#     args = parser.parse_args()

#     tasks_dir = os.path.abspath(args.tasks_dir)
#     logger.info("Using tasks dir: %s", tasks_dir)

#     default_tasks = None
#     if args.tasks:
#         default_tasks = [t.strip() for t in args.tasks.split(",") if t.strip()]

#     # If user wants to use DEFAULT_SEQUENCE hard-coded in this file
#     if args.use_sequence and DEFAULT_SEQUENCE:
#         default_tasks = list(DEFAULT_SEQUENCE)

#     # If user provided a sequence file, load it (overrides discovery but not explicit --tasks)
#     if args.sequence_file:
#         seq_path = os.path.abspath(args.sequence_file)
#         try:
#             with open(seq_path, "r", encoding="utf-8") as fh:
#                 lines = [ln.strip() for ln in fh.readlines()]
#             # filter empty and comment lines
#             seq = [ln for ln in lines if ln and not ln.startswith("#")]
#             if seq:
#                 default_tasks = seq
#         except Exception as e:
#             logger.error("Failed to read sequence file %s: %s", seq_path, e)

#     # Post-manual sequence handling: populate POST_MANUAL_SEQUENCE from defaults or file
#     if args.use_post_sequence and DEFAULT_POST_MANUAL_SEQUENCE:
#         POST_MANUAL_SEQUENCE[:] = list(DEFAULT_POST_MANUAL_SEQUENCE)

#     if args.post_sequence_file:
#         post_path = os.path.abspath(args.post_sequence_file)
#         try:
#             with open(post_path, "r", encoding="utf-8") as fh:
#                 lines = [ln.strip() for ln in fh.readlines()]
#             seq = [ln for ln in lines if ln and not ln.startswith("#")]
#             if seq:
#                 POST_MANUAL_SEQUENCE[:] = seq
#         except Exception as e:
#             logger.error("Failed to read post sequence file %s: %s", post_path, e)

#     # Try to start ROS2 service unless disabled
#     ros_node = None
#     if not args.no_ros:
#         ros_node = try_setup_ros_service(tasks_dir)

#     # If --start provided, run immediately and exit (but keep ROS node alive if present)
#     if args.start:
#         seq = default_tasks if default_tasks is not None else (DEFAULT_SEQUENCE if DEFAULT_SEQUENCE else None)
#         run_sequence(tasks_dir, seq)
#         # If ROS is running, keep process alive to accept service calls
#         if ros_node is not None:
#             logger.info("Keeping process alive to serve ROS2 requests. Ctrl-C to quit.")
#             try:
#                 while True:
#                     time.sleep(1.0)
#             except KeyboardInterrupt:
#                 logger.info("Shutting down")
#         return

#     # Otherwise enter interactive prompt
#     try:
#         _interactive_mode(tasks_dir, default_tasks)
#     except KeyboardInterrupt:
#         print()

#     # Shutdown ROS2 if we started it
#     if ros_node is not None:
#         try:
#             import rclpy
#             logger.info("Shutting down ROS2")
#             rclpy.shutdown()
#         except Exception:
#             pass


# if __name__ == "__main__":
#     main()
#!/usr/bin/env python3
"""
Minimal task sequencer with interactive 'Start' and ROS2 service support.
Runs DEFAULT_SEQUENCE, waits for 'auto' before continuing with POST_MANUAL_SEQUENCE.
"""

import os, sys, threading, subprocess, logging, argparse, time
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger("sequencer")

DEFAULT_SEQUENCE = [
    "pinion_gear_task.py",
    "pinion_spindle_task.py",
    "spur_spindle_task.py",
    "adhesive_applier_task.py",
    "manual_mode_changer.py"
]
DEFAULT_POST_MANUAL_SEQUENCE = [
    "adhesive_returner_task.py",
    "cover_plate_task.py",
    "manual_mode_changer.py"
    ]
POST_MANUAL_SEQUENCE = []
resume_event = threading.Event()
_current_thread = None
_thread_lock = threading.Lock()

def find_tasks(dir): return sorted(f for f in os.listdir(dir) if f.endswith(".py") and not f.startswith("_") and f!="__init__.py")

def run_task_file(py, dir, file):
    path = os.path.join(dir, file)
    if not os.path.isfile(path): return 127
    logger.info("Running %s", file)
    proc = subprocess.Popen([py, path], cwd=dir, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def stream(pipe, prefix=""):
        for line in iter(pipe.readline, ""):
            print(prefix + line.rstrip())
        pipe.close()

    t_out = threading.Thread(target=stream, args=(proc.stdout, ""), daemon=True)
    t_err = threading.Thread(target=stream, args=(proc.stderr, "ERR: "), daemon=True)
    t_out.start()
    t_err.start()
    proc.wait()
    t_out.join()
    t_err.join()
    logger.info("Finished %s (%d)", file, proc.returncode)
    return proc.returncode


def run_sequence(dir, files=None):
    py=sys.executable
    files=list(files) if files else find_tasks(dir)
    if not files: return []
    results=[]
    for f in files:
        code=run_task_file(py,dir,f)
        results.append(code)
        if "manual" in f.lower():
            logger.info("Manual step: waiting for 'auto'")
            resume_event.clear()
            resume_event.wait()
            logger.info("Resuming after manual")
            if POST_MANUAL_SEQUENCE: run_sequence(dir, POST_MANUAL_SEQUENCE)
        if code!=0: logger.warning("Task %s returned %d",f,code)
    return results

def interactive(dir, default_tasks=None):
    global _current_thread
    print("Type 'Start' to run, 'list' to show tasks, 'auto' to resume manual, 'q' to quit")
    while True:
        try: val=input("> ").strip().lower()
        except (EOFError,KeyboardInterrupt): break
        if val in ("q","quit"): break
        if val=="list": print("\n".join(find_tasks(dir))); continue
        if val.startswith("run "): seq=[x.strip() for x in val[4:].split(",")]; val="start"
        if val=="start":
            seq = default_tasks if default_tasks else DEFAULT_SEQUENCE
            with _thread_lock:
                if not _current_thread or not _current_thread.is_alive():
                    _current_thread=threading.Thread(target=run_sequence,args=(dir,seq),daemon=True)
                    _current_thread.start()
                else: print("Sequence already running")
            continue
        if val=="auto": resume_event.set(); print("Resuming"); continue
        print("Unknown command")

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--tasks-dir",default=os.path.abspath(os.path.join(os.getcwd(),"../src/tasks")))
    p.add_argument("--tasks"); p.add_argument("--start",action="store_true")
    p.add_argument("--use-post-sequence",action="store_true")
    args=p.parse_args()
    tasks_dir=os.path.abspath(args.tasks_dir)
    logger.info("Using tasks dir: %s",tasks_dir)
    global POST_MANUAL_SEQUENCE
    if args.use_post_sequence: POST_MANUAL_SEQUENCE=list(DEFAULT_POST_MANUAL_SEQUENCE)
    default_tasks=[t.strip() for t in args.tasks.split(",")] if args.tasks else None
    if args.start: run_sequence(tasks_dir, default_tasks if default_tasks else DEFAULT_SEQUENCE)
    else: interactive(tasks_dir, default_tasks)

if __name__=="__main__": main()

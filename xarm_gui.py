#!/usr/bin/env python3
"""
Simple terminal-like GUI to run a Python script (interactive) using a pseudo-terminal.

Features:
- Default script: DEFAULT_SCRIPT
- Start/Stop the script
- Browse to choose a different script
- Terminal-like output area and an input entry to send text to the process

Works on Linux.
"""
from __future__ import annotations

import os
import subprocess
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext
import tkinter.font as tkfont
import pty
import select
import errno

# Default script path
DEFAULT_SCRIPT = "/home/vmlabs/xarm7_cowork/dev_ws/src/run_xarm7.py"


class TerminalGUI:
	def __init__(self, master: tk.Tk):
		self.master = master
		master.title("Xarm UI")
		# Default window size and allow resizing
		master.geometry("1100x700")
		master.minsize(800, 600)
		master.resizable(True, True)

		# font used by terminal and inputs (adjustable)
		self.ui_font = tkfont.Font(family="Courier", size=11)
		# Bind shortcuts to adjust font size
		master.bind('<Control-plus>', lambda e: self.change_font(1))
		master.bind('<Control-equal>', lambda e: self.change_font(1))
		master.bind('<Control-minus>', lambda e: self.change_font(-1))
		master.bind('<Control-underscore>', lambda e: self.change_font(-1))


		# Top frame: script selection and controls
		top = tk.Frame(master)
		top.pack(fill=tk.X, padx=6, pady=6)

		tk.Label(top, text="Script:").pack(side=tk.LEFT)
		self.path_var = tk.StringVar(value=DEFAULT_SCRIPT)
		self.path_entry = tk.Entry(top, textvariable=self.path_var, width=60)
		self.path_entry.pack(side=tk.LEFT, padx=(6, 4))
		tk.Button(top, text="Browse", command=self.browse).pack(side=tk.LEFT)
		tk.Button(top, text="Run", command=self.start_process).pack(side=tk.LEFT, padx=4)
		tk.Button(top, text="Stop", command=self.stop_process).pack(side=tk.LEFT)

		# Terminal output
		self.text = scrolledtext.ScrolledText(master, state=tk.DISABLED, wrap=tk.NONE, height=24)
		self.text.pack(fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))

		# Input entry
		bottom = tk.Frame(master)
		bottom.pack(fill=tk.X, padx=6, pady=(0, 6))
		self.input_var = tk.StringVar()
		self.input_entry = tk.Entry(bottom, textvariable=self.input_var)
		self.input_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
		self.input_entry.bind('<Return>', self.on_enter)
		tk.Button(bottom, text="Send", command=self.send_input).pack(side=tk.LEFT, padx=4)

		# Process/pty state
		self.proc: subprocess.Popen | None = None
		self.master_fd: int | None = None
		self._reader_thread: threading.Thread | None = None
		self._stop_reader = threading.Event()

		master.protocol("WM_DELETE_WINDOW", self.on_close)

	def browse(self) -> None:
		path = filedialog.askopenfilename(title="Select script", filetypes=[("Python files", "*.py"), ("All files", "*")])
		if path:
			self.path_var.set(path)

	def append_text(self, data: str) -> None:
		self.text.configure(state=tk.NORMAL)
		self.text.insert(tk.END, data)
		self.text.see(tk.END)
		self.text.configure(state=tk.DISABLED)

	def start_process(self) -> None:
		if self.proc and self.proc.poll() is None:
			messagebox.showinfo("Info", "Process is already running")
			return

		script = self.path_var.get().strip() or DEFAULT_SCRIPT
		if not os.path.isfile(script):
			messagebox.showerror("Error", f"Script not found: {script}")
			return

		# Open a new pseudo-terminal pair
		master_fd, slave_fd = pty.openpty()
		# Start the process attached to the slave side of the pty so it behaves like a terminal
		args = ["python3", script]
		try:
			self.proc = subprocess.Popen(
				args,
				stdin=slave_fd,
				stdout=slave_fd,
				stderr=slave_fd,
				close_fds=True,
				preexec_fn=os.setsid,  # give the child its own process group
			)
		except Exception as e:
			os.close(master_fd)
			os.close(slave_fd)
			messagebox.showerror("Error", f"Failed to start process: {e}")
			return

		# we only need the master fd in this process
		os.close(slave_fd)
		self.master_fd = master_fd
		self._stop_reader.clear()
		self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
		self._reader_thread.start()
		self.append_text(f"--- Started: {' '.join(args)} (pid {self.proc.pid}) ---\n")

	def _reader_loop(self) -> None:
		if self.master_fd is None:
			return
		fd = self.master_fd
		try:
			while not self._stop_reader.is_set():
				# Wait until there is data to read or the process exits
				rlist, _, _ = select.select([fd], [], [], 0.1)
				if fd in rlist:
					try:
						data = os.read(fd, 4096)
					except OSError as e:
						if e.errno == errno.EIO:
							# EIO means EOF on some pty drivers
							break
						raise
					if not data:
						break
					try:
						text = data.decode(errors='replace')
					except Exception:
						text = repr(data)
					# Tkinter calls must run on main thread; use `after`
					self.master.after(0, self.append_text, text)
				# Check if process ended
				if self.proc and self.proc.poll() is not None:
					break
		finally:
			# Process ended or we are stopping
			self.master.after(0, self._on_process_exit)

	def _on_process_exit(self) -> None:
		if self.proc:
			rc = self.proc.poll()
			self.append_text(f"\n--- Process exited with return code {rc} ---\n")
		if self.master_fd is not None:
			try:
				os.close(self.master_fd)
			except Exception:
				pass
			self.master_fd = None
		self.proc = None

	def stop_process(self) -> None:
		if not self.proc:
			return
		try:
			# try polite termination
			self.proc.terminate()
		except Exception:
			pass
		self._stop_reader.set()
		# Wait a short time; if still alive, kill
		try:
			self.proc.wait(timeout=1.0)
		except Exception:
			try:
				self.proc.kill()
			except Exception:
				pass

	def send_input(self) -> None:
		text = self.input_var.get()
		if text is None:
			return
		# special toggle: open manual control when user types 'n'
		if text.strip().lower() == 'n':
			# toggle manual controls (don't forward the 'n' to the process)
			self.toggle_manual_controls()
			self.input_var.set("")
			
		# send text plus newline to running process
		data = text + "\n"
		if self.master_fd is None:
			self.append_text("(no process)\n")
			return
		try:
			os.write(self.master_fd, data.encode())
		except OSError as e:
			self.append_text(f"(write error: {e})\n")
		self.input_var.set("")

	def toggle_manual_controls(self) -> None:
		"""Show or hide the manual control panel."""
		if getattr(self, 'manual_frame', None):
			# already visible -> hide
			self.hide_manual_controls()
		else:
			self.create_manual_controls()

	def create_manual_controls(self) -> None:
		"""Create the manual control panel with five service-call buttons."""
		self.master.title("Xarm UI - Manual control activated")
		self.manual_frame = tk.Frame(self.master, bd=2, relief=tk.SUNKEN)
		self.manual_frame.pack(fill=tk.X, padx=6, pady=(0,6))
		label = tk.Label(self.manual_frame, text="Manual control activated", font=(None, 10, 'bold'))
		label.pack(anchor=tk.W, padx=6, pady=(4,2))

		# Define commands
		base = "cd /home/vmlabs/xarm7_cowork/dev_ws ; source install/setup.bash ; "
		buttons = [
			("Start Task Sequence", base + "ros2 service call /start_task_sequence xarm_msgs/srv/Trigger \"{run: true}\""),
			("Start Auto Sequence", base + "ros2 service call /start_auto_sequence xarm_msgs/srv/Trigger \"{run: true}\""),
			("Open Gripper", base + "ros2 service call /start_open_gripper xarm_msgs/srv/Trigger \"{run: true}\""),
			("Go Home", base + "ros2 service call /go_home xarm_msgs/srv/Trigger \"{run: true}\""),
			("Stop Motion", base + "ros2 topic pub /stop_motion xarm_msgs/msg/StopMsg \"{stop_flag: true}\" --once && ros2 topic pub /stop_motion xarm_msgs/msg/StopMsg \"{stop_flag: true}\" --once"),
			("Reset", base + "ros2 service call /reset xarm_msgs/srv/Trigger \"{run: true} \""),
			("relauncher ", base + "ros2 run xarm_sequence_runner process_launcher")
		]

		btn_frame = tk.Frame(self.manual_frame)
		btn_frame.pack(fill=tk.X, padx=6, pady=(0,6))
		for (label_text, cmd) in buttons:
			b = tk.Button(btn_frame, text=label_text, width=20, command=lambda c=cmd, l=label_text: self._run_ros_command(c, l))
			b.pack(side=tk.LEFT, padx=4, pady=4)

		# Add a close button
		close = tk.Button(self.manual_frame, text="Hide Manual", command=self.hide_manual_controls)
		close.pack(anchor=tk.E, padx=6, pady=(0,6))

	def hide_manual_controls(self) -> None:
		if getattr(self, 'manual_frame', None):
			self.manual_frame.destroy()
			self.manual_frame = None
			self.master.title("Xarm UI")

	def _run_ros_command(self, cmd: str, label: str) -> None:
		"""Start a background thread to run the provided shell command and stream output."""
		self.append_text(f"--- Running: {label} ---\n")
		thr = threading.Thread(target=self._ros_command_thread, args=(cmd, label), daemon=True)
		thr.start()

	def change_font(self, delta: int) -> None:
		"""Increase or decrease UI font size by delta."""
		size = int(self.ui_font.cget('size')) + delta
		if size < 6:
			size = 6
		self.ui_font.configure(size=size)

	def _ros_command_thread(self, cmd: str, label: str) -> None:
		# Use bash -lc so `source` works and the chained commands run
		proc = subprocess.Popen(["/bin/bash", "-lc", cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
		if not proc.stdout:
			self.master.after(0, self.append_text, f"(no output from command)\n")
			return
		for line in proc.stdout:
			try:
				text = line.decode(errors='replace')
			except Exception:
				text = repr(line)
			self.master.after(0, self.append_text, text)
		proc.wait()
		self.master.after(0, self.append_text, f"--- {label} finished with code {proc.returncode} ---\n")

	def on_enter(self, event=None) -> str:
		self.send_input()
		return "break"

	def on_close(self) -> None:
		# Stop child process if running
		self._stop_reader.set()
		if self.proc and self.proc.poll() is None:
			try:
				self.proc.terminate()
			except Exception:
				pass
		# give time for threads to finish
		self.master.destroy()


def main() -> None:
	root = tk.Tk()
	gui = TerminalGUI(root)
	root.mainloop()


if __name__ == "__main__":
	main()


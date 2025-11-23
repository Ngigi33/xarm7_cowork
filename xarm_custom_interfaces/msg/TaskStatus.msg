# TaskStatus.msg

string task_name      # e.g., pinion_gear_task.py
string state          # Starting, In-progress, Finished, Error
builtin_interfaces/Time timestamp
float32 progress      # Optional: 0.0 - 100.0
int32 task_id         # optional ID for tracking tasks

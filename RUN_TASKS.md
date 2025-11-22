# How to run tasks (quick guide)

This short guide shows the exact commands you can run to execute tasks (example: `pinion_gear_task.py`) inside the project's virtual environment and the expected output you will see.

Prerequisites
- You have already created the virtual environment `xarm_env` (or equivalent) and installed the SDK into it.

Steps

1) Change to the project folder and activate the venv

```bash
cd xarm7_cowork/dev_ws/src
source xarm_env/bin/activate
```

Expected prompt after activation:

```text
vmlabs@vml-robotics:~/xarm7_cowork$ source xarm_env/bin/activate
(xarm_env) vmlabs@vml-robotics:~/xarm7_cowork$
```

2) Change into the `tasks` folder

```bash
cd tasks/
```

3) Run a task script (replace `pinion_gear_task.py` with any task file)

```bash
python pinion_gear_task.py
```

Sample output observed when running `pinion_gear_task.py`:

```text
(xarm_env) vmlabs@vml-robotics:~/xarm7_cowork/tasks$ python pinion_gear_task.py 
SDK_VERSION: 1.17.0
[2025-11-22 21:22:08][157] xArm-Python-SDK Version:1.17.0
ROBOT_IP: 172.16.40.20, VERSION: v2.5.5, PROTOCOL: V1, DETAIL: 7,7,XS1304,AC1303,v2.5.5, TYPE1300: [1, 1]
change protocol identifier to 3
```


Exit the virtual environment when you're finished:

```bash
deactivate
```


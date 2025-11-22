# Set up and install xArm Python SDK in a virtual environment

Follow these steps to create a Python virtual environment, install the SDK there, and configure the example robot config with your control box IP.

Important: run all commands from a shell (bash) on Linux. Adjust `python3` if your system points to a different executable.

## 1) Create a virtual environment

```bash
python3 -m venv xarm_env
```

This will create a folder named `xarm_env` in the current directory.

## 2) Activate the virtual environment

````bash
source xarm_env/bin/activate
````

Your prompt will change to show the environment name (e.g., `(xarm_env)`).

## 3) Clone the SDK repository

```bash
git clone https://github.com/xArm-Developer/xArm-Python-SDK.git
```

## 4) Install the SDK into the virtual environment

```bash
cd xArm-Python-SDK
python setup.py install
```

Notes:
- If you encounter permission errors, make sure the virtualenv is activated and you are not using `sudo` which installs globally.
- If you prefer pip, you can also run `pip install .` from inside the repo while the venv is active.

## 5) Configure the example robot network

Open the example wrapper configuration file and set the IP of the control box (robot controller):

```bash
cd example/wrapper/
# Use your preferred editor, for example:
nano robot.conf
```

In `robot.conf` change the IP address entry to the IP address of your control box (the robot controller) and save the file.

If you are unsure which IP to use, verify on the controller or the robot's UI/network settings. The control box is the device that runs the robot control and will respond to the SDK.

## 6) When finished

To leave the virtual environment:

```bash
deactivate
```

## Troubleshooting

- If `python3 -m venv` is not available, install the `python3-venv` package for your distribution (e.g., `sudo apt install python3-venv`).
- If `python setup.py install` fails due to a missing build dependency, first try `pip install --upgrade pip setuptools wheel` inside the venv and rerun.
- If the example cannot talk to the robot after setting the IP, ensure network connectivity (ping the IP) and check any firewall settings.

## Quick checklist

- [ ] Create venv: `python3 -m venv xarm_env`
- [ ] Activate: `source xarm_env/bin/activate`
- [ ] Clone repo
- [ ] Install SDK: `python setup.py install`
- [ ] Edit `./example/wrapper/robot.conf` and set control box IP
- [ ] Test example scripts
- [ ] `deactivate` when done

---


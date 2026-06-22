# Lab Laptop Setup: Ubuntu 22.04 + ROS 2 Humble

This is the native, no-Docker setup path for the shared G1 laptop.

Assumptions:

- Ubuntu 22.04 is already installed.
- ROS 2 Humble is already installed at `/opt/ros/humble`.
- Lab member accounts do not need sudo.
- A sudo-capable admin account runs the system setup once.
- The robot network interface is chosen per terminal with `G1_INTERFACE`.

## 1. Admin System Setup

Run once on the laptop from a sudo-capable account:

```bash
cd /path/to/g1pilot
sudo scripts/setup_humble_system_deps.sh --full
```

This installs the shared apt/system dependencies needed by the no-sudo user setup, including the build dependencies for the Docker-equivalent source stacks.

For a lighter admin system setup that skips optional MOLA apt packages:

```bash
sudo scripts/setup_humble_system_deps.sh
```

For joystick access, add each lab user to the `input` group:

```bash
sudo usermod -aG input <username>
```

The user must log out and back in after group changes.

## 2. Per-User Workspace

Each lab member runs this without sudo from their own clone of the lab fork:

```bash
cd /path/to/g1pilot
scripts/setup_humble_user_workspace.sh
```

Default workspace:

```text
~/g1pilot_ws
```

Override it with:

```bash
export G1PILOT_WS=/some/path/g1pilot_ws
scripts/setup_humble_user_workspace.sh
```

The script creates and builds:

- `~/g1pilot_ws/src/g1pilot` as a symlink to the current repo.
- `~/g1pilot_ws/src/astroviz_interfaces`.
- `~/g1pilot_ws/external/unitree_sdk2_python`.
- `~/g1pilot_ws/.venv` with the Python packages needed by this repo.
- `~/g1pilot_ws/deps` with the Docker-equivalent user-space source dependencies.
- `~/g1pilot_ws/env_humble.sh` for future terminals.

The script follows the Docker lessons already recorded in `running_notes.md`:

- build with venv `python -m colcon`.
- keep `setuptools<80`.
- keep `numpy==1.26.4`.
- keep `opencv-python<4.12`.
- install Unitree SDK with pinned prerequisites and `--no-deps`.
- avoid the removed `pin`, `placo`, `pin-pink`, and `playground` packages.

This builds or installs, under `~/g1pilot_ws/deps`, the Docker-equivalent heavy pieces:

- `matlogger2` through `hhcm-forest`.
- `hpp-fcl`.
- Pinocchio pinned to `v3.9.0`.
- `xbot2_interface` with `-DBoost_USE_DEBUG_RUNTIME=OFF`.
- OSQP, proxsuite, FCL `v0.6.0`, qpSWIFT.
- OpenSoT.
- Livox SDK2 and `livox_ros_driver2`.
- Python RealSense package for direct USB/local RealSense utilities.
- Unitree TeleImager client. See "G1 Camera Note" below for the robot-side service.

The script defaults to `JOBS=4` to avoid overloading the laptop during CMake builds. Override it explicitly when appropriate:

```bash
JOBS=8 scripts/setup_humble_user_workspace.sh
```

Useful troubleshooting options:

```bash
scripts/setup_humble_user_workspace.sh --skip-opensot
scripts/setup_humble_user_workspace.sh --skip-livox
scripts/setup_humble_user_workspace.sh --skip-realsense-python
scripts/setup_humble_user_workspace.sh --skip-teleimager-client
```

## G1 Camera Note

For the official Unitree image-stream path, run TeleImager on the robot development computer (usually `192.168.123.164`) and connect to it from the laptop.

On this lab G1, TeleImager is already present at `~/teleimager` in the `teleimager` conda environment. If another robot image does not have it, clone `https://github.com/unitreerobotics/teleimager.git` on PC2 and install the server extras in the robot-side Python environment:

```bash
git clone https://github.com/unitreerobotics/teleimager.git
cd teleimager
pip install -e ".[server]"
```

Start the RealSense-backed image service on PC2:

```bash
conda activate teleimager
cd ~/teleimager
teleimager-server --cf --rs
teleimager-server --rs
```

The lab G1 D435i was validated with serial `348522074178` and `640x480x30`, which corresponds to `image_shape: [480, 640]` in TeleImager's `cam_config_server.yaml`.

From the laptop, use the verified Python client:

```bash
teleimager-client --host 192.168.123.164
```

The WebRTC preview may also be exposed at `https://192.168.123.164:60001`, but the Python client is the path validated during the lab setup.

TeleImager streams camera images over ZMQ/WebRTC. If RViz needs a ROS `PointCloud2` depth cloud, run a ROS RealSense wrapper on the robot computer that is physically connected to the camera.

## 3. Build

After the first setup, rebuild with:

```bash
scripts/build_humble_workspace.sh --packages-up-to g1pilot
```

Or manually:

```bash
source ~/g1pilot_ws/env_humble.sh
cd ~/g1pilot_ws
python -m colcon build --symlink-install --packages-up-to g1pilot
```

## 4. Offline Terminal

For no-robot checks:

```bash
source ~/g1pilot_ws/env_humble.sh
```

Examples:

```bash
ros2 launch g1pilot robot_state_launcher.launch.py use_robot:=false
ros2 launch g1pilot navigation_launcher.launch.py use_robot:=false
```

## 5. Real G1 Terminal

Find the Ethernet interface connected to the G1:

```bash
ip link
```

Then source the G1 network environment:

```bash
source scripts/source_g1_humble.sh <interface>
```

Example:

```bash
source scripts/source_g1_humble.sh eno1
```

For localhost-only DDS tests:

```bash
source scripts/source_g1_humble.sh lo
sudo ip link set lo multicast on
```

Normal lab accounts should not need sudo for the real G1 interface path. `sudo ip link set lo multicast on` is only for localhost DDS tests if loopback multicast is disabled.

## 6. Important Runtime Choices

Real robot launches require a network interface:

```bash
ros2 launch g1pilot robot_state_launcher.launch.py use_robot:=true interface:=$G1_INTERFACE
```

Offline launches should use:

```bash
use_robot:=false
```

The lab G1 reported `mode_machine=5` on `enp134s0`, but its waist roll/pitch joints are physically locked and waist yaw remains free. Set the Unitree-side waist configuration to locked-waist / 1-DOF mode if available, then use a locked-waist URDF in this stack: yaw stays active, roll/pitch must be fixed and uncommanded. See `running_notes.md` for the current validation follow-up.

# G1Pilot Running Notes

Date: 2026-06-21
Workspace: `/home/srinivas/Desktop/g1pilot-workspace/g1pilot`
Repository: `hucebot/g1pilot`

## Current Goal

Set up one clean Docker-based environment for the Unitree G1 lab workflow, then run and validate the parts of the codebase that do not require a connected G1.

## Docker Setup Notes

Build command:

```bash
cd /home/srinivas/Desktop/g1pilot-workspace/g1pilot/docker
sudo docker build --progress=plain -t g1pilot:latest .
```

Local Dockerfile fixes made so far:

- Pinned Pinocchio from live `devel` to `v3.9.0`.
  - Reason: current Pinocchio `devel` has moved to the `coal` collision stack, while this Dockerfile builds `hpp-fcl`.
- Added `-DBoost_USE_DEBUG_RUNTIME=OFF` to the `xbot2_interface` CMake configure line.
  - Reason: Boost.Random configure rejected the previous debug runtime setting on this Ubuntu/Jazzy image.
- Removed `torch` from the base `pip install`.
  - Reason: the repo currently has no `torch` imports, and plain PyPI torch can pull a large shifting dependency set. Install it later only if a real torch-backed path needs it.
- Added a dry-run guard in `g1pilot/navigation/loco_client.py` for `/base_height`.
  - Reason: with `use_robot:=false`, `self.robot` is `None`; the callback previously called `self.robot.SetStandHeight(...)` unconditionally.
- Added missing ROS package dependencies to `package.xml`, including `astroviz_interfaces`.
  - Reason: `robot_state.py` and `dx3_hand.py` import `MotorState`/`MotorStateList`, but `colcon build --packages-select g1pilot` did not build `astroviz_interfaces`.
- Added guarded ROS shutdown handling for the dry-run nodes tested below.
  - Reason: ROS 2 signal handling can already have shut down the context before a node's `finally` block runs.

Current build state:

- Docker image `g1pilot:latest` builds successfully.
- MuJoCo, Pinocchio C++ libraries, OpenSoT, xbot2 interface, Livox ROS driver, Unitree ROS 2 packages, and the later navigation stack dependencies all build/install.
- Livox needed `-DDISTRO_ROS=${ROS_DISTRO}` in the colcon CMake args; the old `-DHUMBLE_ROS=humble` flag did not satisfy Livox's CMake checks on Jazzy.
- A stale `/home/.base/bin/register-python-argcomplete` entry is removed during the build to stop shell-startup tracebacks.
- Venv `setuptools` is kept below 80 where the Dockerfile upgrades pip tooling, because `colcon-core` requires `setuptools<80`.
- Unused `placo`, `playground`, `pin-pink`, and pip Python `pin` installs were removed.
  - `playground` pulled JAX/numpy 2.x.
  - pip Python `pin`/cmeel could import with a custom library path, but globally prepending cmeel libraries made xbot bindings double-free at interpreter shutdown.
  - The repo only used Python Pinocchio for two quaternion helpers, now replaced with SciPy `Rotation` in `g1pilot/utils/helpers.py`.
- Unitree Python SDK is installed with pinned prereqs and `--no-deps` so it does not upgrade numpy.
- The image keeps numpy on `1.26.4` and OpenCV below `4.12`.

Final image audit:

```text
opensot/xbot import group ok
python /home/.base/bin/python
numpy OK 1.26.4 /usr/lib/python3/dist-packages/numpy/__init__.py
scipy OK 1.11.4 /usr/lib/python3/dist-packages/scipy/__init__.py
pyopensot OK /home/forest_ws/install/lib/python3.12/site-packages/pyopensot.cpython-312-x86_64-linux-gnu.so
unitree_sdk2py OK /unitree_sdk2_python/unitree_sdk2py/__init__.py
cyclonedds OK /home/.base/lib/python3.12/site-packages/cyclonedds/__init__.py
cv2 OK 4.6.0 /usr/lib/python3/dist-packages/cv2.cpython-312-x86_64-linux-gnu.so
PIL OK 10.2.0 /usr/lib/python3/dist-packages/PIL/__init__.py
packaging OK 24.0 /usr/lib/python3/dist-packages/packaging/__init__.py
```

Repo-mounted helper audit:

```text
mat_to_quat_wxyz(np.eye(3)) -> [1.0, 0.0, 0.0, 0.0]
quat_wxyz_to_matrix(...) -> identity rotation
```

## No-G1 Run Strategy

The Dockerfile builds system dependencies and support repos, but it does not copy this repo into the image. For local development, mount this repo into the container at `/ros2_ws/src/g1pilot`.

Container build/test pattern:

```bash
sudo docker run --rm --net host \
  -v /home/srinivas/Desktop/g1pilot-workspace/g1pilot:/ros2_ws/src/g1pilot \
  -w /ros2_ws \
  g1pilot:latest \
  bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/.base/bin/activate && /home/.base/bin/python3 -m colcon build --symlink-install --packages-up-to g1pilot'
```

Important: use `/home/.base/bin/python3 -m colcon`, not plain `/usr/bin/colcon`, so generated console scripts use the venv interpreter and can import `unitree_sdk2py`.

Safe node smoke tests should use `use_robot:=false`, for example:

```bash
ros2 run g1pilot loco_client --ros-args -p use_robot:=false
ros2 run g1pilot robot_state --ros-args -p use_robot:=false -p publish_joint_states:=true
```

Smoke tests passed in Docker:

- `astroviz_interfaces` + `g1pilot` build with `--packages-up-to g1pilot`.
- Console-entry imports passed for:
  - `robot_state`, `loco_client`, `create_map`, `dijkstra_planner`, `nav2point`, `mola_fixed`, `joy_mux`, `interactive_marker`, `opensot_solver`, `dx3_controller`, `joystick`, and `ui_interface`.
- Timed no-G1 ROS node runs passed for:
  - `create_map`
  - `dijkstra_planner`
  - `nav2point`
  - `mola_fixed`
  - `robot_state --ros-args -p use_robot:=false -p publish_joint_states:=true`
  - `loco_client --ros-args -p use_robot:=false`
  - `joy_mux`
  - `interactive_marker` waits for TF as expected.
- `/base_height` publish against dry `loco_client` passed; the node stayed alive after the message.

No-G1 caveats:

- `dx3_controller` is not hardware-safe yet; it always initializes Unitree hand DDS.
- `opensot_solver --ros-args -p use_robot:=false` still needs `/robot_state_publisher/get_parameters` to provide `robot_description`.
- `joystick` imports, but actual running depends on available input devices.

Top-level launch files currently require `G1_INTERFACE` even when `use_robot:=false`. For dry launch testing, either set a harmless dummy interface:

```bash
export G1_INTERFACE=lo
```

or use direct `ros2 run` commands until launch cleanup is done.

## Unitree SDK Loco Example vs G1Pilot Loco Client

SDK example:

- File checked: `example/g1/high_level/g1_loco_client_example.py` from `lnotspotl/unitree_sdk2_python` commit `7c661d27f4ae064ffd0dd633fd9d5b518ef0b508`.
- Terminal menu demo.
- Calls `ChannelFactoryInitialize(...)`, creates `LocoClient`, sets timeout, calls `Init()`, then waits for explicit user commands.
- Commands include `Damp`, `Squat2StandUp`, `StandUp2Squat`, one-shot `Move`, `LowStand`, `HighStand`, `ZeroTorque`, `WaveHand`, `ShakeHand`, and `Lie2StandUp`.
- Movement calls are one-shot, e.g. `Move(0.3, 0, 0)`.

G1Pilot `loco_client.py`:

- ROS 2 node wrapping Unitree `LocoClient`.
- Subscribes to `/g1pilot/joy`, `/g1pilot/emergency_stop`, `/g1pilot/start`, `/g1pilot/start_balancing`, and `/base_height`.
- Publishes arm/gripper control topics.
- Adds a balance procedure that raises base height, enters balance mode, then starts locomotion.
- Uses continuous long-duration `Move(..., continous_move=True)` while a joystick button is held.
- Adds dry mode through `use_robot:=false`.

Important differences and risks:

- G1Pilot currently sends robot commands during node startup when `use_robot:=true`; the SDK example waits for explicit user input.
- G1Pilot calls `SetFsmId(4)` before `Init()`, while the SDK example initializes the client before sending actions.
- There is no SDK return-code checking around safety-critical commands.
- Joystick button/axis indexing assumes a specific controller layout.
- Continuous move needs a watchdog if joystick messages stop.
- `nav2point.py` appears to publish autonomous motion on a different button convention than `loco_client.py` consumes.
- Gripper topic/message types appear mismatched between `loco_client.py` and `dx3_hand.py`.

## Torch Decision

Torch is not required by the current repo code paths:

```bash
rg "import torch|from torch|torch\."
```

returned no matches. It can be installed later in the image or container if a model/inference path needs it.

## Repository Strategy

Current local remote:

```text
origin https://github.com/sri299792458/g1pilot.git
upstream https://github.com/hucebot/g1pilot.git
branch dev
```

Status as of the fork setup discussion:

- Personal fork created at `https://github.com/sri299792458/g1pilot`.
- Local branch `dev` has been created.
- `origin` points to the personal fork.
- `upstream` points to `hucebot/g1pilot`.
- The local push URL for `upstream` is disabled to avoid accidental upstream pushes.

Recommended lab workflow:

- Create a lab-owned GitHub organization fork of `hucebot/g1pilot`, not a personal fork.
- Set the lab fork as `origin` for day-to-day work.
- Keep the original repo as `upstream` so upstream fixes can still be fetched and merged/cherry-picked.
- Protect the lab fork's `main` branch and require pull requests for lab members.
- Use a dedicated integration branch for the robot laptop target, e.g. `humble-native` or `lab-humble`, while Docker/Jazzy cleanup can stay on a separate branch if kept.
- Keep robot-specific secrets, network details, calibration captures, and local machine config out of the public code repo. Use ignored local config files or a separate private config repo if needed.
- Only create a brand-new unrelated repo if the lab intends a hard fork with no upstream tracking, or if privacy constraints make a public GitHub fork unsuitable.

## Follow-Up Cleanup Candidates

- Propagate `use_robot` through `bringup_launcher.launch.py` and `bringup_opensot.launch.py`.
- Make `G1_INTERFACE` required only when a node will actually connect to the robot.
- Add a no-hardware launch target that excludes Unitree hand DDS nodes and GUI/RViz if not needed.
- Move `loco_client.py` startup order to `Init()` before `SetFsmId(...)`.
- Add return-code checks and logging for Unitree SDK calls.
- Add a movement watchdog for continuous locomotion.
- Align autonomous joystick button conventions between `nav2point.py`, `joy_mux.py`, and `loco_client.py`.
- Align gripper control topics and message types between `loco_client.py` and `dx3_hand.py`.

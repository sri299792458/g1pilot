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

If `g1pilot:latest` reports `No module named colcon` from `/home/.base/bin/python3 -m colcon`,
the local image tag is older than the current Dockerfile's venv-colcon setup. Rebuild the image.
For launch-description-only checks, `/usr/bin/colcon` can still be used because the nodes are not executed.

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

- `dx3_controller use_robot:=false` does not initialize Unitree hand DDS.
- Standalone `manipulation_launcher.launch.py use_robot:=false` starts `robot_state_publisher` by default so OpenSoT can fetch `robot_description`.
- `joystick` imports, but actual running depends on available input devices.

Top-level launch files no longer require `G1_INTERFACE` when `use_robot:=false`.

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
- `nav2point.py` previously published autonomous motion on a different button convention than `loco_client.py` consumes. This was aligned in issue 31 below.
- Gripper topic/message types between `loco_client.py`, `ui_interface.py`, and `dx3_hand.py` were aligned in issue 12 below.

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
- `dev` has been pushed to `origin/dev`.
- Current setup commit on `dev`: `8db113e` (`Prepare G1Pilot dry-run setup`).

Recommended lab workflow:

- Create a lab-owned GitHub organization fork of `hucebot/g1pilot`, not a personal fork.
- Set the lab fork as `origin` for day-to-day work.
- Keep the original repo as `upstream` so upstream fixes can still be fetched and merged/cherry-picked.
- Protect the lab fork's `main` branch and require pull requests for lab members.
- Use a dedicated integration branch for the robot laptop target, e.g. `humble-native` or `lab-humble`, while Docker/Jazzy cleanup can stay on a separate branch if kept.
- Keep robot-specific secrets, network details, calibration captures, and local machine config out of the public code repo. Use ignored local config files or a separate private config repo if needed.
- Only create a brand-new unrelated repo if the lab intends a hard fork with no upstream tracking, or if privacy constraints make a public GitHub fork unsuitable.

## Follow-Up Cleanup Candidates

- Add return-code checks and logging for Unitree SDK calls.
- Add a movement watchdog for continuous locomotion.

## Static Review Triage

Review copied into `static_review.md`.

Chosen learning-oriented fix order:

1. Package/entrypoints
2. Launch graph
3. Topic graph
4. Robot state / TF
5. Locomotion safety
6. Manipulation / OpenSoT
7. DX3 hands
8. Navigation pipeline
9. Native Humble setup

Issue 1 fixed: package install missed runtime assets.

- `setup.py` now installs `config/*.yaml`, `config/*.json`, `config/*.rviz`, and `pipelines/*.yaml`.
- Verified with container colcon build that installed package contains:
  - `share/g1pilot/config/29dof.rviz`
  - `share/g1pilot/config/config.yaml`
  - `share/g1pilot/config/g1+tracker.rviz`
  - `share/g1pilot/config/livox_mid.json`
  - `share/g1pilot/pipelines/lidar3d.yaml`

Issue 2 fixed: launch files referenced source-checkout paths.

- `robot_state_launcher.launch.py` now resolves `29dof.rviz` from `get_package_share_directory("g1pilot")`.
- `livox_launcher.launch.py` now points the Livox driver at installed `share/g1pilot/config/livox_mid.json`.
- `mola_launcher.launch.py` now defaults `mola_lo_pipeline` to installed `share/g1pilot/pipelines/lidar3d.yaml`.
- Verified in container that:
  - installed package-share assets exist.
  - `ros2 launch g1pilot robot_state_launcher.launch.py --show-args` loads with `G1_INTERFACE=lo`.
  - `ros2 launch g1pilot livox_launcher.launch.py --show-args` loads.
  - `ros2 launch g1pilot mola_launcher.launch.py --show-args` loads and reports the installed pipeline path.

Issue 3 fixed: `g1pilot/tools/test.py` was not packaged.

- Added `g1pilot/tools/__init__.py` so `find_packages()` includes `g1pilot.tools`.
- Did not add a `ros2 run` entrypoint yet because the tool is robot-facing and runs a hanger boot sequence when executed as a script.
- Verified:
  - `find_packages()` includes `g1pilot.tools`.
  - installed overlay can import `g1pilot.tools.test`.
  - `hanger_boot_sequence` is available after import.

Issue 4 fixed: package manifest missed standard runtime dependencies.

- Added `package.xml` runtime dependencies for:
  - `python3-evdev`
  - `python3-numpy`
  - `python3-scipy`
  - `python3-yaml`
  - `robot_state_publisher`
  - `rviz2`
- Existing ROS message/launch dependencies were already present from the dry-run setup work.
- Intentionally did not add Unitree SDK, OpenSoT, xbot, PyQt6, Livox, or MOLA dependencies here yet.
  - Unitree SDK/OpenSoT/xbot are external/native setup concerns.
  - PyQt6 does not resolve through rosdep on this host.
  - Livox and MOLA should be handled in the native Humble setup plan so optional source/apt availability is explicit.
- Verified in container:
  - `catkin_pkg` parses `package.xml`.
  - `colcon build --packages-up-to g1pilot` still succeeds.

Issue 5 fixed: launch files hard-required `G1_INTERFACE` before launch arguments were evaluated.

- Removed immediate `sys.exit()` checks from:
  - `bringup_launcher.launch.py`
  - `bringup_opensot.launch.py`
  - `navigation_launcher.launch.py`
  - `robot_state_launcher.launch.py`
  - `manipulation_launcher.launch.py`
- `interface` now defaults to `EnvironmentVariable("G1_INTERFACE", default_value="")`.
- Added launch-time validation:
  - `use_robot:=true` with no interface raises a clear error.
  - `use_robot:=false` can load without `G1_INTERFACE`.
- Top-level bringup launch files now declare and propagate `use_robot` to navigation, robot state, and manipulation launches.
- Verified in container:
  - Installed launch descriptions show args without `G1_INTERFACE` when `use_robot:=false`.
  - `navigation_launcher.launch.py use_robot:=false` starts `loco_client` and `nav2point` without `G1_INTERFACE`.
  - `navigation_launcher.launch.py use_robot:=true` without `G1_INTERFACE` fails immediately with the new explicit error.
- Follow-up:
  - Full top-level offline launch still needs separate smoke testing because it starts the larger navigation, teleoperation, robot-state, and manipulation graph together.

Issue 6 fixed: `dx3_controller` ignored dry mode.

- `dx3_hand.py` now declares:
  - `use_robot`
  - `send_commands`
- `use_robot:=false` keeps the ROS action subscriptions alive but does not initialize Unitree DX3 DDS channels.
- `send_commands:=false` allows subscribing to hand state without creating command publishers.
- `use_robot:=true` with an empty interface now raises before touching Unitree channels.
- `manipulation_launcher.launch.py` now passes `use_robot` and `send_cmds_to_robot` through to `dx3_controller`.
- Verified in container:
  - Installed overlay can construct `DX3Controller` with `use_robot:=false`.
  - In dry mode, `send_commands` is false and no right/left Unitree command publishers are created.
  - Default robot mode with no interface fails with `interface is required when use_robot:=true`.
- Remaining caveats:
  - DX3 topic/message mismatches are still deferred to the topic-graph pass.

Issue 7 fixed: standalone manipulation launch could hang waiting for `robot_description`.

- `manipulation_launcher.launch.py` now has `start_robot_state_publisher`.
  - Default is `true`, so standalone manipulation launch provides `/robot_state_publisher/get_parameters`.
  - Top-level `bringup_launcher.launch.py` and `bringup_opensot.launch.py` pass `start_robot_state_publisher:=false` because they already include `robot_state_launcher`.
- `opensot_solver.py` now declares `robot_description_timeout_s` with default `10.0`.
  - If `/robot_state_publisher/get_parameters` is unavailable, it raises a clear error instead of waiting forever.
  - If the parameter request does not complete, it raises a clear `robot_description` error.
- Verified in container:
  - `manipulation_launcher.launch.py use_robot:=false` starts `robot_state_publisher`, `opensot_solver`, and `dx3_controller`.
  - OpenSoT initializes past the old missing-service point and builds its model/tasks.
  - Running `opensot_solver` alone with `robot_description_timeout_s:=2.0` exits with the explicit timeout error instead of hanging.

Issue 8 fixed: manipulation launch exposed IK arguments that were not connected to any node.

- Removed unused `manipulation_launcher.launch.py` launch arguments:
  - `enable_arm_ui`
  - `ik_use_waist`
  - `ik_alpha`
  - `ik_max_dq_step`
  - `arm_velocity_limit`
- Kept `arm_controlled` as the explicit arm-side selector.
- Follow-up at the time:
  - True left/right/both arm selection in OpenSoT needed a solver-stack review rather than blindly wiring launch arguments into unrelated solver constants. This is now handled by issue 9 below.

Issue 9 fixed: `arm_controlled` now reaches OpenSoT, not just DX3.

- `opensot_solver.py` now declares and validates `arm_controlled`.
  - Valid values are `left`, `right`, and `both`.
- OpenSoT task-stack behavior now matches the selected side:
  - `right`: `torso_task_3to5 + right_gripper_task`
  - `left`: `torso_task_3to5 + left_gripper_task`
  - `both`: `torso_task_3to5 + right_gripper_task + left_gripper_task`
- OpenSoT only subscribes to pose-reference topics and creates interactive markers for the selected side(s).
- Robot command output only fills selected arm motor commands from the OpenSoT solution.
  - Waist commands are still produced because the current task stack includes torso/base behavior.
- `manipulation_launcher.launch.py`, `bringup_launcher.launch.py`, and `bringup_opensot.launch.py` now propagate `arm_controlled`.
- `dx3_hand.py` now validates `arm_controlled` the same way.
- While validating this, another blocker was fixed:
  - `pyopensot_collision` is now imported optionally.
  - `enable_collision_avoidance:=false` no longer requires the collision package.
  - `enable_collision_avoidance:=true` raises a clear error if `pyopensot_collision` is unavailable.
  - Top-level bringup launch files now default `enable_collision_avoidance` to `false`, matching `manipulation_launcher.launch.py`.
  - Collision avoidance is now opt-in with `enable_collision_avoidance:=true`.
- Verified in container:
  - `opensot_solver.py use_robot:=false arm_controlled:=right enable_collision_avoidance:=false` initializes `torso_task_3to5+right_gripper_task` and only creates the right marker.
  - `opensot_solver.py use_robot:=false arm_controlled:=left enable_collision_avoidance:=false` initializes `torso_task_3to5+left_gripper_task` and only creates the left marker.
  - `opensot_solver.py use_robot:=false arm_controlled:=both enable_collision_avoidance:=false` preserves the original both-arm task stack.
  - `colcon build --symlink-install --packages-up-to g1pilot` succeeds in the container.
  - `ros2 launch g1pilot manipulation_launcher.launch.py use_robot:=false arm_controlled:=left --show-args` loads.
  - `ros2 launch g1pilot bringup_opensot.launch.py use_robot:=false arm_controlled:=right --show-args` loads.

Issue 10 fixed: OpenSoT inserted each interactive marker twice.

- `opensot_solver.py::make_6dof_marker()` had two copies of the same block:
  - add menu control
  - insert marker into `InteractiveMarkerServer`
  - apply the menu
  - apply server changes
- Removed the duplicate second block.
- The function now leaves exactly one menu control and one server insertion per marker.
- Verified:
  - `rg` shows one `menu_control` block and one `self.interactive_marker_server.insert(...)` call in `make_6dof_marker()`.
  - `python3 -m py_compile g1pilot/manipulation/opensot_solver.py` succeeds.
  - Direct no-G1 OpenSoT smoke test with `arm_controlled:=both` initializes both hand markers without tracebacks.

Issue 11 fixed: interactive Cartesian hand-goal topics did not connect to OpenSoT.

- Standardized Cartesian hand goals on the topic names already used by RViz, docs, and `interactive_marker.py`:
  - `/g1pilot/hand_goal/right`
  - `/g1pilot/hand_goal/left`
- `opensot_solver.py` now subscribes to those canonical topics.
- Removed the older runtime topic names from OpenSoT:
  - `/g1pilot/right_hand/pose_ref`
  - `/g1pilot/left_hand/pose_ref`
- Updated `config/config.yaml` to use the canonical topic names for `robot.left_eff_topic` and `robot.right_eff_topic`.
- Verified:
  - `python3 -m py_compile g1pilot/manipulation/opensot_solver.py g1pilot/manipulation/interactive_marker.py` succeeds.
  - Direct no-G1 OpenSoT node-info checks show:
    - `arm_controlled:=both` subscribes to both canonical hand-goal topics.
    - `arm_controlled:=right` subscribes only to `/g1pilot/hand_goal/right`.
    - `arm_controlled:=left` subscribes only to `/g1pilot/hand_goal/left`.
  - Node-info checks show no subscriptions to the old `*_hand/pose_ref` topics.

Issue 12 fixed: DX3 hand command topics and message types were inconsistent.

- Standardized DX3 hand commands on the topic names already used by `loco_client.py` and `docs/CHEATS.md`:
  - `/g1pilot/dx3/hand_action/right`
  - `/g1pilot/dx3/hand_action/left`
- Standardized the command message type on `std_msgs/String`.
- `dx3_hand.py` now subscribes to those canonical String topics.
- `ui_interface.py` now publishes the same String actions to those topics.
- `config/config.yaml` now uses the canonical topic names for `right_hand_topic` and `left_hand_topic`.
- Supported DX3 actions:
  - `open`
  - `pinch`
  - `close`
- Verified:
  - Runtime source scan shows no remaining old DX3 action topics:
    - `/g1pilot/right_hand/dx3/action`
    - `/g1pilot/left_hand/dx3/action`
    - `/right_hand/dx3/action`
    - `/left_hand/dx3/action`
    - `/g1pilot/dx3/right/hand_action`
    - `/g1pilot/dx3/left/hand_action`
  - `python3 -m py_compile g1pilot/manipulation/dx3_hand.py g1pilot/teleoperation/ui_interface.py g1pilot/navigation/loco_client.py` succeeds.
  - Dry `dx3_controller` node-info checks show:
    - `arm_controlled:=both` subscribes to both canonical DX3 String topics.
    - `arm_controlled:=right` subscribes only to `/g1pilot/dx3/hand_action/right`.
    - `arm_controlled:=left` subscribes only to `/g1pilot/dx3/hand_action/left`.
  - Dry command publish test logs:
    - `DX3 right hand action: open`
    - `DX3 left hand action: pinch`
  - `close_1` is rejected with `DX3 hand action must be one of: open, pinch, close`.

Issue 13 fixed: arm enable/home commands were published but not consumed.

- `loco_client.py`, `ui_interface.py`, and `docs/CHEATS.md` publish:
  - `/g1pilot/arms/enabled`
  - `/g1pilot/arms/home`
- `opensot_solver.py` now subscribes to both.
- `/g1pilot/arms/enabled` now enables/disables OpenSoT arm control.
  - The UI button now publishes this canonical topic directly.
  - Removed `/g1pilot/start_opensot`; OpenSoT is an implementation detail, not the command name.
- `/g1pilot/arms/home` now resets selected hand goals to their stored home marker poses.
  - `arm_controlled:=both` resets both hands.
  - `arm_controlled:=right` resets only the right hand.
  - `arm_controlled:=left` resets only the left hand.
- Verified:
  - `python3 -m py_compile g1pilot/manipulation/opensot_solver.py g1pilot/teleoperation/ui_interface.py` succeeds.
  - `git diff --check` succeeds.
  - Direct no-G1 OpenSoT node-info shows subscriptions to:
    - `/g1pilot/arms/enabled`
    - `/g1pilot/arms/home`
  - Direct no-G1 OpenSoT node-info shows no `/g1pilot/start_opensot` subscription.
  - Publishing `/g1pilot/arms/enabled` logs enable/disable messages.
  - Publishing `/g1pilot/arms/home` in `arm_controlled:=both` logs both hand resets.
  - Publishing `/g1pilot/arms/home` in `arm_controlled:=right` logs only the right hand reset.
  - Docker system `colcon build --symlink-install --packages-up-to g1pilot` succeeds.

Issue 14 fixed: `robot_state` could publish malformed `JointState` messages.

- Why this matters:
  - `sensor_msgs/JointState` expects aligned arrays.
  - The previous real-robot callback always kept all 29 joint names, but appended positions only for motor entries present in the incoming Unitree `LowState`.
  - If a short or malformed motor-state packet arrived, `name` and `position` lengths could differ.
- `robot_state.py` now builds `JointState.name` and `JointState.position` from the same observed motor entries.
- Short motor-state packets now publish a valid partial joint state and log one warning.
- While in this node, the already-launched `sim_rate_hz` parameter is now declared and used for the dry-mode timer.
  - Invalid `sim_rate_hz <= 0.0` raises a clear error.
- Verified:
  - `python3 -m py_compile g1pilot/state/robot_state.py` succeeds.
  - Docker focused callback check with a fake 27-motor `LowState` publishes 27 joint names and 27 positions.
  - The same check verifies `sim_rate_hz:=25.0` is applied.
  - Docker system `colcon build --symlink-install --packages-up-to g1pilot` succeeds.

Issue 15 fixed: `robot_state` treated live IMU attitude as a TF mounting transform.

- Why this matters:
  - IMU orientation belongs in `sensor_msgs/Imu`.
  - TF should describe the fixed sensor mounting frame relative to the robot body.
  - The previous code stamped IMU messages as `pelvis`, then published a dynamic `pelvis -> imu_link` transform using the live IMU quaternion.
  - That made TF look like the sensor was rotating relative to the pelvis.
- The URDF already defines the fixed IMU frame as `imu_in_pelvis`.
- `robot_state.py` now publishes `/g1pilot/imu` with `header.frame_id = "imu_in_pelvis"` in both real and dry modes.
- Removed the extra `robot_state` TF broadcaster for `pelvis -> imu_link`.
  - The fixed `pelvis -> imu_in_pelvis` transform is owned by `robot_state_publisher` from the URDF.
- Updated config/RViz references from `imu_link` to `imu_in_pelvis`.
- Verified:
  - `python3 -m py_compile g1pilot/state/robot_state.py` succeeds.
  - Source scan shows no runtime `imu_link` references outside historical `static_review.md`.
  - Docker focused check confirms dry-mode and fake real-mode IMU messages use `imu_in_pelvis`.
  - Docker system `colcon build --symlink-install --packages-up-to g1pilot` succeeds.

Issue 16 fixed: duplicate and false TF publishers in the robot-state/OpenSoT path.

- Why this matters:
  - In ROS TF, each child frame should have one authoritative parent.
  - `robot_state_launcher.py` published `base_link -> pelvis`, but `base_link` is not part of the default URDF tree and the node name implied the opposite direction.
  - The same launcher also published `mid360_link -> livox_frame`, while the default URDF already defines that fixed `livox_joint`.
  - `opensot_solver.py` published a one-shot constructor `world -> pelvis` transform and later published a timestamped dynamic `world -> pelvis` transform in the control loop.
- Removed the launcher static transform publishers for:
  - `base_link -> pelvis`
  - `mid360_link -> livox_frame`
- `robot_state_publisher` now owns robot-internal fixed frames from the URDF.
- Removed the one-shot OpenSoT constructor transform.
  - OpenSoT still publishes the timestamped dynamic `world -> pelvis` transform from the control loop when arm control is enabled.
- Renamed the OpenSoT broadcaster from `base_link_broadcaster` to `tf_broadcaster`.
- Verified:
  - `python3 -m py_compile g1pilot/manipulation/opensot_solver.py launch/robot_state_launcher.launch.py` succeeds.
  - `git diff --check` succeeds.
  - Source scan shows no `robot_state_launcher.py` static transform publishers.
  - Docker `ros2 launch g1pilot robot_state_launcher.launch.py use_robot:=false --show-args` loads.
  - Docker no-G1 OpenSoT initializes and publishes `world -> pelvis` on `/tf` after `/g1pilot/arms/enabled` is set true.
  - Docker system `colcon build --symlink-install --packages-up-to g1pilot` succeeds.
- Follow-up:
  - `fix_mola_odometry.py` still published `map -> pelvis`, which could conflict with OpenSoT's dynamic `world -> pelvis` if both were run as TF authorities for `pelvis`.
  - Resolved in Issue 17 below.

External reference check: official Unitree `unitree_ros` G1 URDFs.

- Cloned as a read-only sparse reference under:
  - `/home/srinivas/Desktop/g1pilot-workspace/reference_repos/unitree_ros`
- Reference commit:
  - `d6f13aa` / `d6f13aad60320ce1d60a07b82a76b5a553f2a0a9`
  - Commit date: `2026-06-18`
  - Subject: `add G1 AGX Backpack`
- Sparse path checked:
  - `robots/g1_description`
- Compared local `description_files/urdf/g1_29dof.urdf` against official 29-DOF variants.
- Important frame observations:
  - Official Unitree 29-DOF URDFs do not define `base_link`.
  - Official Unitree 29-DOF URDFs do not define `imu_link`.
  - Official Unitree 29-DOF URDFs use `pelvis`, `imu_in_pelvis`, `imu_in_torso`, `d435_link`, and `mid360_link`.
  - Official Unitree 29-DOF URDFs do not define `livox_frame`; that is local integration glue for `livox_ros_driver2`.
  - Official Unitree 29-DOF URDFs do not define this repo's OpenSoT helper frames:
    - `left_hand_point_contact`
    - `right_hand_point_contact`
  - Official Unitree 29-DOF URDFs do not define this repo's foot contact helper frames.
- Closest structural match:
  - Local `g1_29dof.urdf` is closest to official `g1_29dof_rev_1_0.urdf`, with local additions for:
    - `world` / `floating_base_joint`
    - foot contact helper frames
    - hand point-contact helper frames
    - `livox_frame`
  - The local file uncommented Unitree's own `world -> pelvis` floating-base block, which Unitree leaves commented as a Mujoco conversion note.
  - This is likely intentional for OpenSoT/XBot free-floating kinematics, but ROS TF ownership still needs to remain explicit.
- Sensor transform comparison against official `g1_29dof_rev_1_0.urdf`:
  - `imu_in_pelvis_joint` matches:
    - `parent=pelvis`, `xyz="0.04525 0 -0.08339"`, `rpy="0 0 0"`
  - `imu_in_torso_joint` matches:
    - `parent=torso_link`, `xyz="-0.03959 -0.00224 0.14792"`, `rpy="0 0 0"`
  - `d435_joint` matches:
    - `parent=torso_link`, `xyz="0.0576235 0.01753 0.42987"`, `rpy="0 0.8307767239493009 0"`
  - `mid360_joint` does not match official `rev_1_0` / mode 11-16 files:
    - Local:
      - `xyz="0.0002835 0.00003 0.41618"`
      - `rpy="0 0.04014257279586953 0"`
    - Official:
      - `xyz="0.0002835 0.00003 0.428434"`
      - `rpy="3.141592653589793 0.05112069379091391 0"`
- Other model differences:
  - Local ankle and waist limits use higher `effort` / `velocity` values than official `rev_1_0`.
  - Those limits may be local control/modeling choices and should not be changed without checking the actual robot mode and controller assumptions.
- Conclusion:
  - The Unitree repo is the primary reference for ROS URDF frame names.
  - Do not use the earlier NVIDIA/GR00T exploration to drive this ROS URDF cleanup unless we later do a separate sim-asset migration.
  - It confirms our prior cleanup away from invented `base_link` and `imu_link` TFs.
  - The next URDF issue worth understanding is the `mid360_joint` transform mismatch, because it directly affects lidar point-cloud alignment and mapping.

GitHub search: forks and copied repos with URDF/frame changes.

- Searched GitHub forks of `hucebot/g1pilot`:
  - 49 forks.
  - 107 total branches across forks.
  - 12 branches touched `description_files/urdf/g1_29dof.urdf`.
- Searched non-fork repositories named like `g1pilot`:
  - `VectorRobotics/G1Pilot`
  - `SAKErobotics/g1pilot`
  - `ybkimws-source/g1pilot_ybkim`
- Searched GitHub code for distinctive copied-URDF strings:
  - `right_hand_point_contact_fix_joint`
  - local `mid360_joint` value `0.0002835 0.00003 0.41618`
  - official Unitree `mid360_joint` value `0.0002835 0.00003 0.428434`
  - `livox_joint mid360_link`
- Copied/derived URDFs inspected locally under:
  - `/home/srinivas/Desktop/g1pilot-workspace/reference_repos/_raw_candidates`
- Findings:
  - `SAKErobotics/g1pilot`, `adrialemany/g1man`, `hucebot/huro`, `SimonR99/unitree_g1_ros2`, `YoheiHayamizu/unitree_g1_stack`, `yusongmin1/go2_flip_TO`, `upatras-lar/se3_trajopt`, and `Ojasva-Goyal/test` mostly preserve the same local `mid360_joint` transform as g1pilot:
    - `xyz="0.0002835 0.00003 0.41618"`
    - `rpy="0 0.04014257279586953 0"`
  - None of the copied g1pilot-derived URDFs inspected had both this repo's contact-frame additions and the official Unitree `rev_1_0` `mid360_joint` transform:
    - `xyz="0.0002835 0.00003 0.428434"`
    - `rpy="3.141592653589793 0.05112069379091391 0"`
  - `KaliberAI/g1pilot` branch `km-nav-tests` has commit `a0634d6` with message `fixed auto_enable + lidar orientation`.
    - Its URDF change is only:
      - `livox_joint` `rpy="3.141561 0 0"` -> `rpy="0 0 0"`
    - It does not update `mid360_joint` to official Unitree `rev_1_0`.
  - `NilsMeier1812/g1pilot` / copied branch family includes relevant world-frame commits:
    - `1ba9d20` `change_opensot_world_definition_with_opensot_foot_point_contact`
    - `a296c31` `change_world_publishing`
    - `9bb481b` `fix_world_frame_bug_for_opensot_sim_and_irl_robot`
    - Those commits make `robot_state.py` compute/broadcast `world -> pelvis` from the left foot contact for the real robot, and make OpenSoT publish `world -> pelvis` only when `use_robot` is false.
    - This is relevant to the remaining TF ownership follow-up; it is not a direct `mid360_joint` geometry fix.
  - `VectorRobotics/G1Pilot` is not a copy of this ROS Python stack. It is a distinct C++ Pinocchio/CasADi manipulation repo.
    - It has a `g1_29dof_with_hand_rev_1_0_ros_ctrl_viz.urdf` that re-roots a visualization/control model under `base_link`.
    - That appears to support a dual `robot_state_publisher` visualization setup and should not be copied into this repo's main URDF without a deliberate architecture change.
- Conclusion:
  - The fork/copy search did not find a better third-party `mid360_joint` URDF fix than official Unitree.
  - It did find a useful precedent for the next TF issue: real-robot `world -> pelvis` should come from state/localization/foot-contact ownership, while OpenSoT should not also publish it in real mode.

Specific check: `ybkimws-source/g1pilot_ybkim`.

- Repo:
  - `https://github.com/ybkimws-source/g1pilot_ybkim`
  - Non-fork GitHub repo.
  - Default branch: `master`.
  - Single visible commit at inspection time:
    - `26846e9` `g1pilot_ybkim: ROS2 manipulation tasks for Unitree G1`
- Cloned read-only for inspection under:
  - `/home/srinivas/Desktop/g1pilot-workspace/reference_repos/g1pilot_ybkim`
- What it is:
  - A task-layer ROS 2 package meant to run inside/alongside hucebot `g1pilot`.
  - Adds pot-pick/lift/reset/gesture demos, gripper visualization, and a `trajectory_solver.py` wrapper with minimum-jerk Cartesian interpolation.
  - Does not ship a full copied `description_files/urdf/g1_29dof.urdf`.
- URDF-related note in its README:
  - It recommends an upstream g1pilot patch that halves arm/wrist velocity limits for simulation:
    - shoulder / elbow / wrist_roll: `37 -> 18.5 rad/s`
    - wrist_pitch / wrist_yaw: `22 -> 11 rad/s`
  - This is not a `mid360_joint`, `base_link`, `imu_link`, or `world -> pelvis` frame fix.
- Useful local observations:
  - `sim_gripper_state.launch.py` loads `g1_29dof_dx3.urdf` from g1pilot and strips duplicate `pelvis_contour_joint` at runtime with regex before passing the URDF to `robot_state_publisher`.
  - That confirms the DX3 URDF duplicate-joint problem exists in downstream use.
- Compatibility with this cleanup branch:
  - It assumes older g1pilot topics:
    - `/g1pilot/start_opensot`
    - `/g1pilot/right_hand/pose_ref`
    - `/g1pilot/left_hand/pose_ref`
    - `/g1pilot/right_hand/dx3/action`
    - `/g1pilot/left_hand/dx3/action`
  - It uses `geometry_msgs/PointStamped` for DX3 hand commands.
  - This branch now uses:
    - `/g1pilot/arms/enabled`
    - `/g1pilot/hand_goal/right`
    - `/g1pilot/hand_goal/left`
    - `/g1pilot/dx3/hand_action/right`
    - `/g1pilot/dx3/hand_action/left`
    - `std_msgs/String` DX3 actions: `open`, `pinch`, `close`
  - `trajectory_solver.py` subclasses `G1CollisionAvoidanceNode`, but it overrides old callback names/fields such as `start_opensot_callback`, `right_hand_pose_ref_callback`, `left_hand_pose_ref_callback`, and `right_hand_pose_ref`.
  - Current `opensot_solver.py` uses `arms_enabled_callback`, `right_hand_goal_callback`, `left_hand_goal_callback`, `right_hand_goal`, and `left_hand_goal`.
- Conclusion:
  - `g1pilot_ybkim` is useful for task sequencing and minimum-jerk Cartesian interpolation ideas.
  - It should not be treated as a URDF/frame reference for this cleanup.
  - It would need a topic/API adapter before running on this branch.

Issue 17 fixed: `pelvis` TF ownership is now explicit for sim vs real robot runs.

- Bug:
  - OpenSoT and localization could both publish a moving transform for the robot body.
  - OpenSoT used the solver state to publish `world -> pelvis`.
  - `fix_mola_odometry.py` used corrected MOLA odometry to publish `map -> pelvis`.
  - If both are active on the real robot, TF can receive competing poses for the same child frame `pelvis`.
- Desired ownership:
  - In no-G1 simulation/visualization, OpenSoT may publish `world -> pelvis` so RViz has a floating-base pose.
  - On the real robot, body pose should come from state estimation/localization, not from the arm IK solver.
- Changes:
  - `opensot_solver.py` now has a `publish_pelvis_tf` parameter.
    - Default is `true` when `use_robot:=false`.
    - Default is `false` when `use_robot:=true`.
  - OpenSoT only sends `world -> pelvis` through `_publish_pelvis_tf()` when `publish_pelvis_tf` is enabled.
  - `fix_mola_odometry.py` now has a `publish_tf` parameter.
    - It can publish corrected odometry without also publishing `map -> pelvis`.
  - `robot_state_launcher.launch.py` exposes `mola_fixed_publish_tf`, default `true`, and passes it to `mola_fixed`.
  - `mola_launcher.launch.py` now applies `MOLA_LOCALIZATION_PUBLISH_TF=False` before the MOLA node group starts.
  - `fix_mola_odometry.py` log text now describes the actual compatibility correction:
    - y position sign flip
    - quaternion w sign flip
- Remaining navigation caution:
  - `fix_mola_odometry.py` still applies a compatibility pose correction and copies twist/covariance unchanged.
  - That should be revisited when we deliberately work through the navigation pipeline.
- Verified:
  - `python3 -m py_compile g1pilot/manipulation/opensot_solver.py g1pilot/navigation/fix_mola_odometry.py launch/robot_state_launcher.launch.py launch/mola_launcher.launch.py` succeeds.
  - `git diff --check -- g1pilot/manipulation/opensot_solver.py g1pilot/navigation/fix_mola_odometry.py launch/robot_state_launcher.launch.py launch/mola_launcher.launch.py` succeeds.
  - Docker `colcon build --symlink-install --packages-up-to g1pilot` succeeds after installing the missing venv packages in the ephemeral container:
    - `colcon-common-extensions`
    - `lark`
    - `decorator`
  - Docker launch-description checks load:
    - `robot_state_launcher.launch.py use_robot:=false --show-args`
    - `manipulation_launcher.launch.py use_robot:=false arm_controlled:=left --show-args`
    - `bringup_opensot.launch.py use_robot:=false arm_controlled:=right --show-args`
    - `mola_launcher.launch.py --show-args`
  - Runtime OpenSoT `/tf` sampling was stopped because the current `g1pilot:latest` venv is still missing runtime packages such as `scipy`.
    - This is Docker image drift, not a source syntax/build failure.

Issue 18 fixed: local LiDAR URDF mount did not match official Unitree G1 `rev_1_0`.

- Bug:
  - The local URDF placed `mid360_link` at:
    - `xyz="0.0002835 0.00003 0.41618"`
    - `rpy="0 0.04014257279586953 0"`
  - Official Unitree G1 `rev_1_0` 29-DOF URDF places `mid360_link` at:
    - `xyz="0.0002835 0.00003 0.428434"`
    - `rpy="3.141592653589793 0.05112069379091391 0"`
  - That is not just a small pitch tweak:
    - z changes by about 12.254 mm.
    - pitch changes by about 0.63 deg.
    - roll changes by 180 deg, which represents the factory inverted MID360 mount convention.
- Important frame detail:
  - Unitree's official URDF stops at `mid360_link`.
  - This repo also has a `livox_frame` because `livox_launcher.launch.py` publishes point clouds with `frame_id='livox_frame'`.
  - Keeping the old `livox_joint` roll after adopting the official inverted `mid360_joint` would apply the inversion twice.
- Changes:
  - Updated `mid360_joint` to the official Unitree `rev_1_0` pose in:
    - `description_files/urdf/g1_29dof.urdf`
    - `description_files/urdf/g1_29dof_dx3.urdf`
    - `description_files/urdf/g1_29dof_upperbody.urdf`
    - `description_files/urdf/g1_29dof_dx3_upperbody.urdf`
  - Changed `livox_joint` from `rpy="3.141561 0 0"` to `rpy="0 0 0"` in the URDF variants that define `livox_frame`.
    - `livox_frame` is now a compatibility alias for `mid360_link`.
- Remaining navigation caution:
  - `mola_launcher.launch.py` currently defaults `ignore_lidar_pose_from_tf:=true`, so MOLA assumes a fixed/origin LiDAR pose instead of reading this URDF TF.
  - This fix corrects the robot TF tree and RViz point-cloud placement, but MOLA extrinsics still need a deliberate navigation pass.
- Verified:
  - Python XML parsing succeeds for all four local URDF variants.
  - All four local `mid360_joint` origins match the official Unitree `rev_1_0` values.
  - Local `livox_joint` origins, where present, are identity transforms from `mid360_link` to `livox_frame`.
  - `git diff --check -- description_files/urdf/g1_29dof.urdf description_files/urdf/g1_29dof_dx3.urdf description_files/urdf/g1_29dof_upperbody.urdf description_files/urdf/g1_29dof_dx3_upperbody.urdf` succeeds.

Live robot LiDAR/TF validation, 2026-06-21:

- Laptop/G1/Livox network:
  - Laptop Ethernet IP: `192.168.123.99`
  - G1 controller reachable at `192.168.123.161`
  - Livox Mid360 reachable at `192.168.123.120`
- `config/livox_mid.json` was updated so the Livox host IP fields match this laptop (`192.168.123.99`).
- Livox driver launches cleanly and publishes `/livox/lidar` as `sensor_msgs/msg/PointCloud2` with `frame_id: livox_frame`.
- Numeric TF confirmed:
  - Command: `ros2 run tf2_ros tf2_echo torso_link livox_frame`
  - Observed:
    - translation `[0.000, 0.000, 0.428]`
    - RPY radians `[3.142, 0.051, 0.000]`
    - RPY degrees `[180.000, 2.929, 0.000]`
  - This matches the expected official `mid360_joint` pose (`z` about `0.428434`, roll about `pi`, pitch about `0.0511 rad`).
- RViz note:
  - If `robot_state_publisher` is launched by itself, fixed links like `livox_frame` are OK but movable links show RobotModel transform errors.
  - Starting `g1pilot robot_state` with `publish_joint_states:=true` publishes `/joint_states` and restores dynamic `/tf` for the full model.
  - `config/29dof.rviz` now keeps the Livox display readable by default:
    - `Color Transformer: AxisColor`
    - `Axis: Z`
    - `Style: Flat Squares`
    - `Size (m): 0.02`
    - `Decay Time: 1`
  - RViz uses `Size (m)` for `Flat Squares`; `Size (Pixels)` only applies to `Style: Points`.
- Repeatable replay bag captured:
  - `/home/kanth042/g1pilot_ws/bags/livox_tf_live_20260621_212824`
  - Contains `/livox/lidar` (99 messages), `/tf` (195 messages), and `/tf_static` (1 message).

Remaining visual RViz check:

- Fixed frame: `pelvis` or `torso_link`
- Add `/livox/lidar` as `PointCloud2`
- Floor should appear below the robot.
- Walls/vertical objects should look vertical.
- A hand moved in front of the LiDAR should appear in front of the robot, not behind or upside down.

Replay/caveat:

- Replay the captured bag against old vs new URDF TFs to compare the correction without repeatedly touching the robot.
- MOLA currently defaults `ignore_lidar_pose_from_tf:=true`, so this validates URDF/RViz/TF first.
- MOLA LiDAR extrinsics need a separate navigation pass.

Issue 19 fixed: `g1_29dof_dx3.urdf` had a duplicate `pelvis_contour_joint`.

- Bug:
  - `description_files/urdf/g1_29dof_dx3.urdf` defined `pelvis_contour_joint` twice.
  - URDF joint names must be unique.
  - Duplicate names can break `robot_state_publisher` and downstream TF generation.
- Context:
  - `ybkimws-source/g1pilot_ybkim` worked around this by stripping the duplicate joint at runtime before launching `robot_state_publisher`.
  - That confirms this was not just a theoretical parser complaint.
- Change:
  - Removed the first duplicate `pelvis_contour_joint`.
  - Kept the joint after `pelvis_contour_link`, matching the structure of the main `g1_29dof.urdf`.
- Verified:
  - Python XML parsing succeeds for every local URDF file.
  - Every local URDF now has unique link and joint names.
  - `git diff --check -- description_files/urdf/g1_29dof_dx3.urdf` succeeds.

Issue 20 fixed: `g1_29dof_dx3_upperbody.urdf` used machine-local mesh paths.

- Bug:
  - `description_files/urdf/g1_29dof_dx3_upperbody.urdf` had 63 mesh filenames under:
    - `/home/cdonoso/Desktop/ROS/g1pilot/description_files/meshes/...`
  - That only works on the original author's filesystem.
  - On another lab account or robot laptop, `robot_state_publisher`/RViz would not be able to resolve those meshes.
- Change:
  - Replaced those absolute mesh paths with:
    - `package://g1pilot/description_files/meshes/...`
  - Normalized this URDF to LF line endings.
    - The file was already mixed CRLF/LF after targeted edits.
    - Keeping CRLF on changed lines made `git diff --check` report trailing whitespace.
- Verified:
  - No local URDF has absolute mesh filenames.
  - Every `package://g1pilot/...` mesh reference in the local URDF files points to an existing file in this repo.
  - Python XML parsing succeeds for every local URDF file.
  - `git diff --check -- description_files/urdf/g1_29dof_dx3_upperbody.urdf` succeeds.

Issue 21 fixed: DX3 URDF variants missed integration helper frames.

- Bug:
  - `description_files/urdf/g1_29dof_dx3.urdf` and `description_files/urdf/g1_29dof_dx3_upperbody.urdf` did not define:
    - `left_hand_point_contact`
    - `right_hand_point_contact`
    - `livox_frame`
  - OpenSoT hard-codes the hand Cartesian task frames:
    - `left_hand_point_contact`
    - `right_hand_point_contact`
  - `livox_launcher.launch.py` publishes point clouds with `frame_id='livox_frame'`.
  - Without these frames, switching from the default rubber-hand URDF to a DX3 URDF would break TF lookup and/or OpenSoT model construction.
- Change:
  - Added `livox_frame` as an identity fixed child of `mid360_link` in both DX3 URDF variants.
  - Added `left_hand_point_contact` as a fixed child of `left_hand_palm_link` in both DX3 URDF variants.
  - Added `right_hand_point_contact` as a fixed child of `right_hand_palm_link` in both DX3 URDF variants.
  - The contact-frame offset matches the existing rubber-hand convention:
    - `xyz="0.05 0.0 0.0"`
    - `rpy="0 0 0"`
- Verified:
  - All local `g1_29dof*.urdf` variants define:
    - `left_hand_point_contact`
    - `right_hand_point_contact`
    - `mid360_link`
    - `livox_frame`
  - Python XML parsing succeeds for every local URDF file.
  - Every local URDF has unique link and joint names.
  - `git diff --check -- description_files/urdf/g1_29dof_dx3.urdf description_files/urdf/g1_29dof_dx3_upperbody.urdf` succeeds.

Issue 22 fixed: launch files could not select the cleaned URDF variants.

- Bug:
  - `robot_state_launcher.launch.py` and `manipulation_launcher.launch.py` hard-coded:
    - `g1_29dof.urdf`
  - That meant the DX3 and upper-body URDF variants could be cleaned up, but not selected through the normal launch surface.
  - Top-level bringup launch files also had no way to keep robot state and manipulation on the same selected URDF.
- Change:
  - Added a `urdf_file` launch argument to:
    - `robot_state_launcher.launch.py`
    - `manipulation_launcher.launch.py`
    - `bringup_launcher.launch.py`
    - `bringup_opensot.launch.py`
  - Allowed packaged URDF choices:
    - `g1_29dof.urdf`
    - `g1_29dof_dx3.urdf`
    - `g1_29dof_upperbody.urdf`
    - `g1_29dof_dx3_upperbody.urdf`
  - The default remains `g1_29dof.urdf`.
  - Launchers validate that `urdf_file` is a packaged file name, not an arbitrary path.
- Usage examples:
  - `ros2 launch g1pilot robot_state_launcher.launch.py urdf_file:=g1_29dof_dx3.urdf`
  - `ros2 launch g1pilot bringup_launcher.launch.py urdf_file:=g1_29dof_dx3.urdf`
  - `ros2 launch g1pilot manipulation_launcher.launch.py use_robot:=false urdf_file:=g1_29dof_dx3.urdf`
- Notes:
  - This does not wire `config/config.yaml` `robot.model` yet.
  - The launch argument is the safer first step because it is explicit at runtime and works for standalone launch files.
- Verified:
  - `python3 -m py_compile launch/robot_state_launcher.launch.py launch/manipulation_launcher.launch.py launch/bringup_launcher.launch.py launch/bringup_opensot.launch.py` succeeds.

Issue 23 fixed: OpenSoT emergency-stop branch was unreachable.

- Bug:
  - `opensot_solver.py` guarded the entire control loop with:
    - `self.arms_enabled and not self.emergency_stop`
  - Later inside that same block it checked:
    - `if self.emergency_stop or not self.motors_on`
  - When `/g1pilot/emergency_stop` was true, the outer guard skipped the whole block, so the intended passive/safe low-command branch could not run.
  - Practically, the solver stopped producing new commands, but it did not actively send the existing passive arm command path.
- Change:
  - Added `_send_passive_arm_command()`.
  - `control_loop()` now checks emergency stop first.
  - If emergency stop is active and the real-robot low-command publisher is initialized, OpenSoT sends the passive arm command for both arm chains and returns before running the solver.
    - This matches the original intended emergency branch scope:
      - `G1_29_JointArmIndex`
      - motors `15-28`
  - Normal solving still requires `/g1pilot/arms/enabled`.
  - `motors_on == false` reuses the same passive command helper.
- Notes:
  - In `use_robot:=false`, emergency stop remains a no-op for hardware commands because there is no Unitree low-command publisher.
  - The OpenSoT emergency helper does not cover waist motors `12-14`, legs `0-11`, or DX3 hand motors.
  - This does not change the broader G1 safety story; hardware testing still needs a deliberate lab procedure.
- Verified:
  - `python3 -m py_compile g1pilot/manipulation/opensot_solver.py` succeeds.
  - `git diff --check -- g1pilot/manipulation/opensot_solver.py` succeeds.

TODO update: lab G1 waist configuration identified before finalizing URDF and waist control.

- Goal:
  - Choose the robot model and waist command policy from both the reported Unitree machine type and the actual physical waist configuration.
- Result:
  - On the lab laptop connected over Ethernet (`enp134s0`, `192.168.123.99/24`), the robot responded at `192.168.123.161`.
  - Direct read of `rt/lowstate.mode_machine` returned:
    - `5`
  - The user confirmed the lab G1 has physically locked waist roll/pitch.
  - Waist yaw remains unlocked, as expected for G1 locked-waist / 1-DOF waist operation.
- Other ways to check:
  - Unitree app:
    - `Device -> Data -> Robot -> Machine Type`
  - Unitree lowstate:
    - read `rt/lowstate.mode_machine`
    - `opensot_solver.py` already reads this field through `get_mode_machine()`
- Why it matters:
  - Unitree publishes different G1 URDFs for different `mode_machine` IDs.
  - The official Unitree `unitree_ros/robots/g1_description` README maps mode `5` to:
    - `g1_29dof_rev_1_0`
    - `g1_29dof_with_hand_rev_1_0`
    - `g1_29dof_rev_1_0_with_inspire_hand_DFQ`
    - `g1_29dof_rev_1_0_with_inspire_hand_FTP`
  - The locked-waist rev 1.0 variant is mode `6`, not mode `5`.
  - In the official locked-waist URDF:
    - `waist_yaw_joint` remains `revolute`.
    - `waist_roll_joint` is `fixed`.
    - `waist_pitch_joint` is `fixed`.
    - The movable joint count drops from 29 to 27.
  - The official reference repo also has newer locked-waist variants for IDs such as:
    - `6`
    - `12`
    - `14`
    - `16`
  - The current local g1pilot URDF set does not yet include these locked-waist variants.
- Revised code follow-up:
  - Set the Unitree-side waist configuration to locked-waist / 1-DOF mode in Unitree Explore if available, then re-check `mode_machine`.
  - Use the locked-waist URDF for this lab robot because the physical roll/pitch waist joints are locked, even if the reported machine type remains `5`.
  - Add the matching official locked-waist URDF variant to `description_files/urdf`.
  - Reapply this repo's integration helper frames:
    - `livox_frame`
    - `left_hand_point_contact`
    - `right_hand_point_contact`
  - Add launch/runtime validation so selected URDF, live `lowstate.mode_machine`, and explicit physical waist configuration cannot silently mismatch.
  - Update OpenSoT waist handling:
    - allow `waist_yaw_joint` / motor `12`.
    - do not command physically locked waist roll/pitch motors `13` and `14`.
    - avoid assuming motor index equals model joint index when fixed joints are removed from the URDF.

Issue 24 fixed: OpenSoT initialized from hard-coded `q_init` instead of live robot state.

- Bug:
  - `opensot_solver.py` read current motor positions into `self.all_motor_q` during Unitree interface initialization.
  - `initialize()` then ignored those values and set:
    - `self.q[7:] = q_init`
  - On the real robot, the first enabled OpenSoT command could therefore target the hard-coded nominal posture instead of the robot's actual current arm/waist posture.
- Change:
  - Imported the existing `JOINT_NAMES_ROS` motor-index to joint-name map.
  - Built a model joint-name to `q`-index map after creating the XBot model.
  - In `use_robot:=true`, initialized mapped model joints from live `LowState` motor positions.
  - In offline mode, or for unavailable values, initialized mapped model joints from `q_init`.
  - Left unmapped model joints at zero and logged a warning.
  - Updated outgoing arm/waist command extraction to use the same model joint-name map instead of assuming:
    - `q_index = 7 + motor_index`
- Why this matters for locked-waist follow-up:
  - A locked-waist URDF can remove waist roll/pitch from the active model joint list.
  - With the new map, absent/fixed joints are skipped instead of being indexed incorrectly.
  - This is not the full locked-waist migration yet; the matching official URDF still needs to be imported after the lab G1 `mode_machine` is identified.
- Verified:
  - `python3 -m py_compile g1pilot/manipulation/opensot_solver.py` succeeds.
  - `git diff --check -- g1pilot/manipulation/opensot_solver.py` succeeds.

Issue 25 fixed: OpenSoT collision debug markers were skipped in offline mode.

- Investigation note:
  - Static review suspected `solver.solve()` output might need multiplication by `control_dt`.
  - Upstream OpenSoT reference check says no code change is needed there:
    - `constraints::velocity::VelocityLimits` bounds the solver variable to `qdot_limit * dT`.
    - OpenSoT examples update state as `q = model->sum(q, dq)` directly.
  - Reference clone:
    - `/home/srinivas/Desktop/g1pilot-workspace/reference_repos/OpenSoT`
    - commit `f1d8c733`
  - So `dq` in this velocity-task stack is a bounded per-step model increment, not a raw velocity that should be multiplied by `control_dt` again.
- Bug fixed:
  - `opensot_solver.py` published collision distance markers only after the real-robot command branch.
  - With `use_robot:=false`, `control_loop()` returned before marker publication.
  - That made collision visualization unavailable in the no-G1/RViz workflow, exactly where it is most useful for debugging.
- Change:
  - Moved `publishCollisionDistances(...)` before the `if not self.use_robot: return` branch.
  - Markers still publish only when `enable_collision_avoidance:=true` created a collision constraint.
- Verified:
  - `python3 -m py_compile g1pilot/manipulation/opensot_solver.py` succeeds.
  - `git diff --check -- g1pilot/manipulation/opensot_solver.py` succeeds.

Issue 26 fixed: DX3 motor-state topics were relative.

- Bug:
  - `dx3_hand.py` published:
    - `g1pilot/dx3/left/motor_state`
    - `g1pilot/dx3/right/motor_state`
  - Because these names had no leading `/`, ROS 2 treated them as relative to the node namespace.
  - That can silently move the topics if the node is launched under a namespace.
  - The rest of the cleaned g1pilot control contract uses explicit `/g1pilot/...` topics.
- Change:
  - Added named constants for the DX3 motor-state topics.
  - State publishers now use:
    - `/g1pilot/dx3/left/motor_state`
    - `/g1pilot/dx3/right/motor_state`
- Verified:
  - `python3 -m py_compile g1pilot/manipulation/dx3_hand.py` succeeds.
  - `git diff --check -- g1pilot/manipulation/dx3_hand.py running_notes.md` succeeds.

Issue 27 fixed: `arm_gui.py` used PyQt5 while the repo uses PyQt6.

- Bug:
  - `g1pilot/utils/arm_gui.py` imported:
    - `from PyQt5 import QtWidgets, QtCore`
  - The active UI code and Docker/package setup use PyQt6.
  - On a clean lab install with only PyQt6, importing this utility would fail even though the rest of the UI dependencies are present.
- Change:
  - Switched the import to:
    - `from PyQt6 import QtWidgets, QtCore`
  - The file already used PyQt6-compatible enum names, so no other code changes were needed.
- Verified:
  - `python3 -m py_compile g1pilot/utils/arm_gui.py` succeeds.
  - `rg -n "PyQt5|PyQt6" g1pilot setup.py package.xml docker -S` shows only PyQt6 imports.
  - `git diff --check -- g1pilot/utils/arm_gui.py running_notes.md` succeeds.

Issue 28 hardened: `loco_client.py` tolerates short `Joy` messages.

- Risk:
  - `loco_client.py` assumes the joystick message has the expected original-author layout.
  - It previously indexed fixed fields directly:
    - `msg.axes[4]`
    - `msg.buttons[5]`
    - `msg.buttons[7]`
    - motion axes `0`, `1`, and `2`
  - The normal in-repo publishers are expected to send enough axes/buttons.
  - A lab joystick, mux, test publisher, or autonomous publisher with fewer fields could still raise `IndexError`.
  - The exception handler then stops/damps the robot and marks it unbalanced, which is a harsh response to a bad message shape.
- Change:
  - Added guarded joystick accessors:
    - `_axis(...)`
    - `_button(...)`
  - Missing axes/buttons now read as neutral values.
  - Short `Joy` messages log one warning until a correctly sized message arrives.
  - Button edge tracking now keeps the expected button range even when a short message arrives.
- Notes:
  - We initially deferred this to avoid unnecessary divergence from original behavior.
  - Re-applied it after deciding the neutral-default behavior is preferable for lab testing.
  - This does not solve joystick semantic mapping; that still needs a shared button/axis contract between `joystick.py`, `nav2point.py`, and `loco_client.py`.
- Verified:
  - `python3 -m py_compile g1pilot/navigation/loco_client.py` succeeds.
  - `git diff --check -- g1pilot/navigation/loco_client.py running_notes.md` succeeds.
  - `rg -n 'msg\\.buttons\\[[0-9]|msg\\.axes\\[[0-9]' g1pilot/navigation/loco_client.py` finds no remaining direct fixed-index accesses.

Issue 29 fixed: balancing failure could still start locomotion and mark the robot balanced.

- Bug:
  - `loco_client.py::entering_balancing()` could log:
    - `Problems during balancing, stopping...`
  - but then still execute:
    - `BalanceStand(1)`
    - `SetStandHeight(...)`
    - `Start()`
    - `self.balanced = True`
  - The same happened if the balancing loop exited because `robot_stopped` was already true.
  - That made a failed/interrupted balancing attempt look successful to the rest of `loco_client.py`.
- Change:
  - `entering_balancing()` now returns `True` only after the successful `BalanceStand/Start` path.
  - If FSM mode reports a balancing problem, it returns `False` before `BalanceStand/Start`.
  - If the robot is stopped before/during the balancing loop, it returns `False`.
  - If the required start height is not reached, it returns `False`.
  - Callers now log completed vs failed based on the return value.
- Scope:
  - This preserves the original successful balancing command sequence.
  - It does not change joystick button layout assumptions.
  - It does not yet add rate limiting or make balancing asynchronous; those are separate follow-ups.
- Verified:
  - `python3 -m py_compile g1pilot/navigation/loco_client.py` succeeds.
  - `git diff --check -- g1pilot/navigation/loco_client.py running_notes.md` succeeds.

Issue 30 fixed: `loco_client.py` sent an FSM command before initializing the SDK client.

- Bug:
  - Startup did:
    - create `LocoClient()`
    - `SetTimeout(...)`
    - `SetFsmId(4)`
    - `Init()`
    - `Damp()`
  - The Unitree SDK example pattern initializes the client before sending action commands.
  - This repo's own `g1pilot/tools/test.py` follows that same order:
    - `SetTimeout(...)`
    - `Init()`
    - later `SetFsmId(...)`
- Change:
  - Moved the startup `SetFsmId(4)` after `self.robot.Init()`.
  - Kept the original standby/damping behavior:
    - `Init()`
    - `SetFsmId(4)`
    - `Damp()`
- Verified:
  - `python3 -m py_compile g1pilot/navigation/loco_client.py` succeeds.
  - `git diff --check -- g1pilot/navigation/loco_client.py running_notes.md` succeeds.

Issue 31 fixed: autonomous `Joy` commands used the wrong walk-enable button.

- Bug:
  - `nav2point.py` published autonomous `Joy` commands to:
    - `/g1pilot/auto_joy`
  - `joy_mux.py` relays `/g1pilot/auto_joy` to:
    - `/g1pilot/joy`
    - when `/g1pilot/auto_enable` is true.
  - `loco_client.py` only calls `Move(...)` when:
    - `msg.buttons[7] == 1`
  - But `nav2point.py` set:
    - `buttons[8] = 1`
  - So autonomous path following could publish velocity axes without satisfying the locomotion walk-enable condition.
- Change:
  - `nav2point.py` now sets:
    - `buttons[7] = 1`
  - `config/config.yaml` now documents the active autonomous topic as:
    - `/g1pilot/auto_joy`
    - instead of stale `/g1pilot/joy_autonomous`
- Verified:
  - `python3 -m py_compile g1pilot/navigation/nav2point.py g1pilot/teleoperation/joy_mux.py g1pilot/navigation/loco_client.py` succeeds.
  - `git diff --check -- g1pilot/navigation/nav2point.py config/config.yaml running_notes.md` succeeds.
  - Source scan shows the active path now agrees:
    - `nav2point.py`: `buttons[7] = 1`
    - `joy_mux.py`: subscribes `/g1pilot/auto_joy`
    - `loco_client.py`: moves on `msg.buttons[7] == 1`

Issue 32 fixed: `nav2point.py` generated auto commands while auto mode was disabled.

- Bug:
  - `nav2point.py::loop()` only used `auto_enabled` around some warning conditions.
  - If a path existed while `/g1pilot/auto_enable` was false, it could still compute and publish a nonzero `Joy` command on:
    - `/g1pilot/auto_joy`
  - `joy_mux.py` normally blocks that while auto is disabled, but it still stores the last auto message.
  - When auto is enabled later, a stale auto command can be relayed until a fresh `nav2point.py` update arrives.
- Change:
  - `nav2point.py::loop()` now returns immediately when `auto_enabled` is false.
  - When auto mode transitions from true to false, `nav2point.py` publishes one neutral auto `Joy` message.
    - This clears the mux's stored auto command without relying on the next control-loop tick.
  - Goal-reached stop output now uses the same neutral `Joy` helper.
- Verified:
  - `python3 -m py_compile g1pilot/navigation/nav2point.py g1pilot/teleoperation/joy_mux.py` succeeds.
  - `git diff --check -- g1pilot/navigation/nav2point.py running_notes.md` succeeds.

Issue 33 fixed: `joy_mux.py` manual-priority window did not override auto commands.

- Bug:
  - `joy_mux.py` computed:
    - `use_manual = ... manual_priority_window ...`
  - But the loop checked auto first:
    - if auto was enabled and `last_auto` existed, publish auto and return.
  - That made `manual_priority_window` ineffective whenever auto mode was active.
- Change:
  - `joy_mux.py::loop()` now checks `use_manual` first.
  - Recent manual input wins for the configured priority window.
  - Auto still publishes when auto is enabled and no recent manual input is active.
- Scope:
  - This only fixes the intended priority ordering.
  - Stale-auto expiration is handled separately in issue 34 below.
- Verified:
  - `python3 -m py_compile g1pilot/teleoperation/joy_mux.py` succeeds.
  - `git diff --check -- g1pilot/teleoperation/joy_mux.py running_notes.md` succeeds.

Issue 34 fixed: `joy_mux.py` could replay stale auto commands indefinitely.

- Bug:
  - `joy_mux.py` stored the latest auto `Joy` message in `last_auto`.
  - If `/g1pilot/auto_enable` stayed true and the auto publisher stalled or died, the mux kept republishing that last auto command forever.
  - Because `loco_client.py` sends continuous locomotion commands while the walk-enable button is held, replaying stale auto `Joy` is unsafe.
- Change:
  - Added `auto_timeout`, default `0.25 s`.
  - `cb_auto()` records the receipt time for each auto `Joy`.
  - `loop()` only relays auto commands while the most recent auto message is fresh.
  - If auto is enabled but the last auto command is stale, the mux publishes a neutral stop `Joy`.
  - The timeout logs once until a fresh auto message arrives.
- Notes:
  - `auto_timeout <= 0.0` disables the timeout and preserves the old replay behavior.
  - This does not replace a lower-level locomotion watchdog; it only prevents stale `/g1pilot/auto_joy` replay in the mux.
- Verified:
  - `python3 -m py_compile g1pilot/teleoperation/joy_mux.py` succeeds.
  - `git diff --check -- g1pilot/teleoperation/joy_mux.py running_notes.md` succeeds.

Issue 35 fixed: `nav2point.py` accepted zero velocity limits.

- Bug:
  - `nav2point.py` clamps commanded velocities with:
    - `vx_limit`
    - `vy_limit`
    - `wz_limit`
  - It then normalizes those clamped values into joystick axes by dividing by the same limits.
  - If any limit was configured as `0`, the control loop hit division-by-zero inside its broad exception handler and stopped producing useful autonomous commands.
- Change:
  - Added startup validation for:
    - `publish_rate`
    - `vx_limit`
    - `vy_limit`
    - `wz_limit`
  - These parameters must now be finite positive numbers.
  - `main()` now shuts ROS down cleanly if node construction fails before `Nav2Point` is assigned.
- Reasoning:
  - In this implementation, a zero velocity limit is not a safe way to disable an axis because the value is also the normalization denominator.
  - Failing at startup is clearer than repeatedly logging loop errors while auto navigation silently does nothing useful.
- Verified:
  - `python3 -m py_compile g1pilot/navigation/nav2point.py g1pilot/teleoperation/joy_mux.py g1pilot/navigation/loco_client.py` succeeds.
  - `git diff --check -- g1pilot/navigation/nav2point.py running_notes.md` succeeds.

Issue 36 fixed: `dijkstra_planner.py` treated unknown or failed planning as drivable.

- Bug:
  - ROS `OccupancyGrid` uses `-1` for unknown cells.
  - `dijkstra_planner.py` only treated `255` as a special unknown sentinel.
  - During inflation, unknown `-1` cells were copied into an all-free `0` grid unless they were also inflated by an obstacle.
  - If the map was missing, the start/goal was outside the map, the start/goal was occupied, or Dijkstra failed, the planner published a straight-line fallback path anyway.
- Change:
  - Unknown cells are now blocked for planning:
    - standard ROS `-1`
    - legacy/internal `255`
  - Inflated occupancy grids preserve unknown cells instead of converting them to free space.
  - Planning failures now publish an empty path and warn instead of publishing an unverified straight line.
  - The existing `occ_threshold` and `inflation_radius_m` parameters are now used for occupancy checks and inflation.
- Reasoning:
  - Publishing no path is safer than publishing a direct path through space the map cannot prove is free.
  - This is still not a full navigation-stack fix:
    - frame transforms are still assumed.
    - diagonal corner cutting is still unchanged.
    - smoothing is still not collision-checked after the final curve.
- Verified:
  - `python3 -m py_compile g1pilot/navigation/dijkstra_planner.py` succeeds.
  - A mocked import check confirms `-1`, `255`, and occupied cells are blocked, while known-free cells remain free.

Issue 37 fixed: navigation launch did not start the path planner.

- Bug:
  - README documents `navigation_launcher.launch.py` as the way to run the navigation stack.
  - The launcher started:
    - `loco_client`
    - `nav2point`
  - It did not start:
    - `dijkstra_planner`
  - So a goal on `/g1pilot/goal` could not produce `/g1pilot/path` from the default navigation launch.
- Change:
  - `navigation_launcher.launch.py` now starts `dijkstra_planner`.
- Scope:
  - Did not automatically start `create_map.py`.
  - `create_map.py` is a dummy map publisher, so launching it by default on the real G1 would make the planner trust synthetic map data.
  - For offline demos, run the dummy map explicitly or provide a real `/map` source.
- Verified:
  - `python3 -m py_compile launch/navigation_launcher.launch.py` succeeds.
  - `git diff --check -- launch/navigation_launcher.launch.py running_notes.md` succeeds.
  - Source scan confirms `navigation_launcher.launch.py` starts `loco_client`, `nav2point`, and `dijkstra_planner`.

Issue 38 fixed: goal publishing docs/RViz did not match the planner input.

- Bug:
  - `dijkstra_planner.py` subscribes to `/g1pilot/goal` as `geometry_msgs/msg/PoseStamped`.
  - `docs/CHEATS.md` still showed a `PointStamped` command.
  - RViz `SetGoal` still published to `/goal_pose`, so the planner would not receive RViz goals.
- Change:
  - Updated the cheat sheet goal command to publish `PoseStamped` on `/g1pilot/goal`.
  - Updated both RViz configs so `SetGoal` publishes to `/g1pilot/goal`.
  - Changed the goal display from `PointStamped` to `Pose`.
- Verified:
  - Source scan confirms no remaining `/goal_pose` topic in the RViz configs.
  - Source scan confirms the cheat sheet uses `PoseStamped` for `/g1pilot/goal`.

Native Humble laptop setup files added.

- Goal:
  - Support the shared lab laptop running Ubuntu 22.04 and ROS 2 Humble without Docker.
- Added:
  - `scripts/setup_humble_system_deps.sh`
    - one-time sudo/admin system dependency setup.
  - `scripts/setup_humble_user_workspace.sh`
    - one no-sudo per-user full setup: workspace, venv, Unitree Python SDK, `astroviz_interfaces`, Docker-equivalent source dependencies, and ROS builds.
  - `scripts/build_humble_workspace.sh`
    - rebuild helper for the per-user workspace.
  - `scripts/source_g1_humble.sh`
    - per-terminal ROS/CycloneDDS/G1 interface environment.
  - `docs/LAB_LAPTOP_SETUP_HUMBLE.md`
    - step-by-step native laptop setup guide.
- Scope:
  - Lab member accounts do not need sudo after the admin has run `setup_humble_system_deps.sh --full`.
  - The member setup script follows the Docker notes:
    - `setuptools<80`
    - `numpy==1.26.4`
    - `opencv-python<4.12`
    - Unitree SDK installed with pinned prerequisites and `--no-deps`
    - Pinocchio pinned to `v3.9.0`
    - `xbot2_interface` built with `-DBoost_USE_DEBUG_RUNTIME=OFF`
    - Livox built with `-DDISTRO_ROS=${ROS_DISTRO}`
    - removed `pin`, `placo`, `pin-pink`, and `playground` remain excluded.
- Verified:
  - `bash -n` succeeds for all new shell scripts.
  - `shellcheck` reports no issues when available.
  - Direct trailing-whitespace scan reports no issues for the new setup scripts and Humble setup doc.

Native Humble setup simplified to one member script.

- Reason:
  - The previous split made the full Docker-equivalent setup look optional.
  - For the real G1 workflow, OpenSoT, Livox/MOLA, Unitree SDK, and related source dependencies are part of the expected setup.
- Change:
  - `scripts/setup_humble_user_workspace.sh` is now the single no-sudo member entrypoint.
  - It performs the full user-space setup by default.
  - Removed `scripts/setup_humble_full_user_deps.sh`.
  - Updated `docs/LAB_LAPTOP_SETUP_HUMBLE.md` to show one member command.
- Notes:
  - `--skip-opensot`, `--skip-livox`, and `--skip-realsense-python` remain as troubleshooting options, not the default lab setup.
  - `--skip-teleimager-client` is available if the Unitree image-stream client should not be installed on the laptop.

G1 RealSense / camera access check, 2026-06-21:

- Final postmortem:
  - The initial RealSense access failure was a real error, but the root cause was the original physical USB port/link path on PC2.
  - It was not caused by missing ROS RealSense packages, TeleImager installation, serial config, normal-user permissions, or the laptop-side setup.
  - Evidence from the bad port:
    - Minimal `pyrealsense2` tests could start or enumerate intermittently but could not reliably receive frames.
    - Kernel logs showed USB/UVC failures on the old path, including `error -71`, failed UVC probe/control queries, resets, and failed device initialization.
  - Evidence after moving the D435i to another USB port:
    - The direct normal-user `pyrealsense2` color pipeline at `640x480x30` succeeded.
    - Output included `got color: True`.
  - Conclusion: the primary blocker was physical USB link instability, not the software stack.
- Official Unitree camera path:
  - Use Unitree TeleImager on PC2 for the G1 head camera video stream.
  - `unitreerobotics/xr_teleoperate` points to `unitreerobotics/teleimager`.
  - TeleImager supports RealSense cameras and streams frames over ZMQ and WebRTC.
- Laptop-side setup:
  - `unitreerobotics/teleimager` is installed editable under `~/g1pilot_ws/external/teleimager`.
  - `scripts/setup_humble_user_workspace.sh` installs the TeleImager client by default.
  - Working laptop viewer:
    - `teleimager-client --host 192.168.123.164`
- PC2 validated camera details:
  - PC2 TeleImager env: `/home/unitree/miniconda3/envs/teleimager`
  - PC2 TeleImager source: `/home/unitree/teleimager`
  - D435i serial: `348522074178`
  - Working RealSense profile: `640x480x30`
  - TeleImager config should use:
    - `head_camera.type: realsense`
    - `head_camera.serial_number: '348522074178'`
    - `head_camera.image_shape: [480, 640]`
    - `head_camera.fps: 30`
    - `head_camera.enable_zmq: true`
    - `head_camera.zmq_port: 55555`
    - `head_camera.enable_webrtc: true`
    - `head_camera.webrtc_port: 60001`
- Current validated state:
  - TeleImager starts on PC2 with the RealSense after using the stable USB port and `640x480x30` profile.
  - Laptop-side ZMQ subscribe to `tcp://192.168.123.164:55555` receives valid JPEG frames.
  - The official Python client reports about `30 FPS`.
  - Browser WebRTC at `https://192.168.123.164:60001` still shows black; this is a separate WebRTC/browser playback issue, not a camera-capture issue.
- Boundary:
  - TeleImager is the official Unitree image/video stream path for teleoperation.
  - It is not currently a ROS `sensor_msgs/msg/PointCloud2` publisher for RViz depth clouds.
  - If the task specifically needs `/camera/camera/depth/color/points`, a ROS RealSense wrapper still has to run on the robot computer physically connected to the D435i.

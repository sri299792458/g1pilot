# Static Review Notes

I completed a thorough static review of the uploaded repository. The Python files compile syntactically, but I could not run full ROS 2 integration or hardware tests in this environment because the robot/ROS-specific dependencies and hardware are not available. I also won’t claim mathematical certainty that there are zero remaining defects, but these are the high-confidence bugs and breakages I found.

## Most serious breakages

1. **Navigation does not work end-to-end by default.** `navigation_launcher.launch.py` starts `loco_client` and `nav2point`, but not `dijkstra_planner` or `create_map`, so publishing `/g1pilot/goal` will not produce `/g1pilot/path`.

2. **Autonomous walking commands never reach locomotion correctly.** `nav2point.py` sets `buttons[8] = 1`, while `loco_client.py` only walks when `buttons[7] == 1`.

3. **DX3 hand control topics and message types are inconsistent.** The DX3 node subscribes to `PointStamped` on `/g1pilot/right_hand/dx3/action` and `/g1pilot/left_hand/dx3/action`, while joystick/docs publish `String` on `/g1pilot/dx3/hand_action/right|left`, and the UI publishes `PointStamped` to `/right_hand/dx3/action` and `/left_hand/dx3/action` without the `/g1pilot` prefix.

4. **Interactive Cartesian hand goals do not connect to OpenSoT.** `interactive_marker.py` publishes `/g1pilot/hand_goal/right|left`, but `opensot_solver.py` subscribes to `/g1pilot/right_hand/pose_ref` and `/g1pilot/left_hand/pose_ref`.

5. **Launch files hard-require `G1_INTERFACE` even for simulation/offline modes.** Several launch files call `sys.exit()` if the environment variable is missing, even when `use_robot:=false` could otherwise be valid.

6. **Packaging is broken for installed deployments.** RViz configs are stored in `config/*.rviz`, but `setup.py` installs `rviz/*.rviz`; JSON and pipeline config files used by launch files are not installed.

7. **OpenSoT cannot be disabled from collision avoidance cleanly.** `enable_collision_avoidance` is declared but ignored; collision avoidance is imported, constructed, and added to the stack unconditionally.

8. **URDF variants are broken.** `g1_29dof_dx3.urdf` has a duplicate joint name, and DX3 URDF variants are missing the `left_hand_point_contact` / `right_hand_point_contact` frames that OpenSoT expects.

9. **TF tree has multiple conflicting parents for `pelvis`.** `robot_state_launcher`, `fix_mola_odometry`, and `opensot_solver` can all publish transforms involving `pelvis` from different parents.

10. **Docker/compose workflow is not reproducible as written.** The compose file references a missing `env.conf`, the healthcheck waits on a topic that is not normally published at startup, and the Dockerfile mixes Jazzy/Humble flags.

---

## Packaging and dependency bugs

1. `setup.py:47-51` installs only `config/*.yaml` and `rviz/*.rviz`. The repository actually stores RViz files in `config/29dof.rviz` and `config/g1+tracker.rviz`, so installed packages will not contain the RViz configs.

2. `setup.py` does not install `config/livox_mid.json` or `pipelines/lidar3d.yaml`, even though `livox_launcher.launch.py` and `mola_launcher.launch.py` rely on these configuration assets.

3. `setup.py:53` has `install_requires=['setuptools']` only. It omits runtime Python dependencies used throughout the code, including `rclpy`, `numpy`, `scipy`, `PyQt6`, `evdev`, `pinocchio`, `xbot2_interface`, `pyopensot`, `unitree_sdk2py`, and `astroviz_interfaces`.

4. `package.xml` has only test dependencies. It lacks `<buildtool_depend>ament_python</buildtool_depend>` and runtime dependencies for ROS packages such as `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2_ros`, `visualization_msgs`, `interactive_markers`, `robot_state_publisher`, and `rviz2`.

5. `g1pilot/tools/test.py` is not packaged by `find_packages()` because `g1pilot/tools` has no `__init__.py`. That file will not be installed as a Python package module.

6. `g1pilot/utils/arm_gui.py` imports `PyQt5`, while the rest of the UI uses `PyQt6` and the Dockerfile installs `pyqt6`. That utility will fail unless PyQt5 is installed separately.

---

## Launch-file bugs

1. `bringup_launcher.launch.py`, `bringup_opensot.launch.py`, `navigation_launcher.launch.py`, `manipulation_launcher.launch.py`, and `robot_state_launcher.launch.py` call `sys.exit()` if `G1_INTERFACE` is unset. Passing `interface:=...` on the launch command line or `use_robot:=false` does not bypass this.

2. `robot_state_launcher.launch.py:95-102` launches RViz from the absolute source path `/ros2_ws/src/g1pilot/config/29dof.rviz`, not from the installed package share directory.

3. `navigation_launcher.launch.py` launches only `loco_client` and `nav2point`. It does not launch `dijkstra_planner`, so no path is produced from goals.

4. `navigation_launcher.launch.py` does not launch `create_map` either, so there is no default `/map` source despite the planner requiring one for obstacle-aware planning.

5. `bringup_launcher.launch.py` does not include `livox_launcher.launch.py` or `mola_launcher.launch.py`, even though the README describes MOLA/Livox as part of the integrated navigation stack.

6. `robot_state_launcher.launch.py` launches `mola_fixed` but not the MOLA odometry producer. `mola_fixed` subscribes to `/lidar_odometry/pose`, which will not exist unless MOLA is launched elsewhere.

7. `manipulation_launcher.launch.py` declares `ik_use_waist`, `ik_alpha`, `ik_max_dq_step`, `arm_velocity_limit`, `arm_controlled`, and `enable_arm_ui`, but only a subset is passed to `opensot_solver`. Most of those launch arguments are currently ignored.

8. `robot_state_launcher.launch.py` passes `sim_rate_hz` to `robot_state`, but `robot_state.py` never declares or uses that parameter.

9. `manipulation_launcher.launch.py` does not launch `interactive_marker`, even though the README/RViz workflow implies interactive Cartesian manipulation.

10. Launching `manipulation_launcher.launch.py` by itself can hang: `opensot_solver.py` waits forever for `/robot_state_publisher/get_parameters`, but the manipulation launch file does not start `robot_state_publisher`.

11. `mola_launcher.launch.py` sets `MOLA_LOCALIZATION_PUBLISH_TF=False` after the `node_group` action. That environment variable may be applied too late for the MOLA node that needs it.

12. `livox_launcher.launch.py` hard-codes `/ros2_ws/src/livox_ros_driver2/config/MID360_config.json` instead of using the package’s own installed `config/livox_mid.json`.

13. `mola_launcher.launch.py` hard-codes `/ros2_ws/src/g1pilot/pipelines/lidar3d.yaml` instead of resolving it from the installed package share directory.

---

## Topic and message integration bugs

1. `dx3_hand.py` subscribes to:

   * `/g1pilot/right_hand/dx3/action`
   * `/g1pilot/left_hand/dx3/action`

   as `geometry_msgs/PointStamped`.

   `loco_client.py` publishes:

   * `/g1pilot/dx3/hand_action/right`
   * `/g1pilot/dx3/hand_action/left`

   as `std_msgs/String`.

   These will never connect.

2. `docs/CHEATS.md` documents DX3 commands as `std_msgs/String` on `/g1pilot/dx3/hand_action/right`, which no DX3 controller subscribes to.

3. `ui_interface.py` publishes DX3 hand actions to `/left_hand/dx3/action` and `/right_hand/dx3/action`, missing the `/g1pilot` prefix required by `dx3_hand.py`.

4. `ui_interface.py` sends `point.x = 0.0` for open and `point.x = 1.0` for close. `dx3_hand.py` interprets `point.x > 0.5` as open, `point.x < -0.5` as partial close, and everything else as full close. The UI open/close semantics are reversed or wrong.

5. `interactive_marker.py` publishes Cartesian goals on `/g1pilot/hand_goal/right` and `/g1pilot/hand_goal/left`.

6. `opensot_solver.py` subscribes instead to `/g1pilot/right_hand/pose_ref` and `/g1pilot/left_hand/pose_ref`.

7. `docs/CHEATS.md` publishes a left-hand pose to `/g1pilot/hand_goal/left`, which OpenSoT never receives.

8. RViz configs display `/g1pilot/hand_goal/left` and `/g1pilot/hand_goal/right`, again not the OpenSoT subscriber topics.

9. `/g1pilot/arms/enabled` and `/g1pilot/arms/home` are published by `loco_client.py` and `ui_interface.py`, and documented in `docs/CHEATS.md`, but no node subscribes to them. Those commands currently do nothing.

10. `dijkstra_planner.py` subscribes to `/g1pilot/goal` as `PoseStamped`, while `docs/CHEATS.md` publishes a `PointStamped` to `/g1pilot/goal`.

11. RViz displays `/g1pilot/goal` as `PointStamped`, but the planner expects `PoseStamped`.

12. The RViz “SetGoal” tool publishes to `/goal_pose`, while `dijkstra_planner.py` listens on `/g1pilot/goal`.

13. `config/config.yaml` defines topic names that do not match the code, for example DX3 topics and `navigation.autonomous_topic: /g1pilot/joy_autonomous` while the code uses `/g1pilot/auto_joy`.

14. `config/config.yaml` is mostly not loaded by the nodes. Editing it will not affect most runtime behavior.

---

## Navigation bugs

1. `dijkstra_planner.py` declares many parameters but ignores them:

   * `inflation_radius_m` is ignored; `0.40` is hard-coded.
   * `occ_threshold` is ignored; `50` is hard-coded.
   * `simplify_min_dist` is ignored; `0.02` is hard-coded.
   * `smooth_samples_per_segment` is ignored; `8` is hard-coded.
   * `smooth_closed` is ignored; `False` is hard-coded.
   * `smooth_enable` is ignored; smoothing always runs.
   * `shortcut_enable` is ignored; shortcutting always runs.
   * `straight_steps` is ignored; `50` is hard-coded.
   * `allow_diagonal` is ignored; diagonal neighbors are always added.

2. OccupancyGrid unknown cells are normally `-1`, but `dijkstra_planner.py` treats only `255` as special unknown. Unknown space is therefore treated as free space.

3. `inflate_occupancy()` initializes all cells to `0`, so unknown cells are actively converted into free cells.

4. `neighbors()` allows diagonal moves through blocked corners. It checks only the diagonal destination cell, not the two adjacent orthogonal cells.

5. The planner does not transform goal, odometry, or map frames. It assumes all coordinates are already in the same frame.

6. If the map is missing, the start/goal are out of bounds, the start/goal are occupied, or Dijkstra fails, the planner publishes a straight-line path anyway. That can command the robot through obstacles.

7. The Catmull-Rom smoothing step is not collision-checked after smoothing, so a previously valid grid path can be curved into obstacles.

8. The turn-cost Dijkstra state includes heading in the queue, but `dist` and `visited` are keyed only by `(x, y)`. With nonzero turn cost, this can discard the lower-cost heading state for future expansion.

9. `line_points()` accepts `frame_id` but never uses it.

10. `create_map.py` declares an `obstacles` parameter but never reads or applies it.

11. `Nav2Point.loop()` publishes auto joystick commands even when `auto_enabled` is false. It only relies on `joy_mux` to filter them.

12. `Nav2Point.loop()` can publish commands with the default pose `(0,0,0)` if a path exists but no odometry has arrived and `auto_enabled` is false.

13. `Nav2Point` divides by `vx_limit`, `vy_limit`, and `wz_limit` without guarding against zero values.

14. `Nav2Point` sets `buttons[8] = 1`, but `loco_client.py` requires `buttons[7] == 1` to move.

15. `Nav2Point` clears its path when the goal is reached, but it does not publish `auto_enable=false` or any goal-reached event.

16. `JoyMux.loop()` gives auto priority before checking recent manual input, so `manual_priority_window` does not actually override auto when `last_auto` exists.

17. `JoyMux` never expires stale auto joystick messages. If `auto_enabled` remains true, it can keep publishing the last auto command indefinitely.

18. `JoyMux` contains pure-pursuit/path-following helper functions, but the active loop only relays `/g1pilot/auto_joy`. The path-following logic is effectively dead code.

---

## Locomotion and joystick bugs

1. `loco_client.py:138-140` calls `self.robot.SetStandHeight()` unconditionally. In `use_robot:=false`, `self.robot` is `None`, so any `/base_height` message crashes the callback.

2. `opensot_solver.py` publishes `/base_height`; combined with `loco_client use_robot:=false`, starting OpenSoT can trigger the crash above.

3. `loco_client.py` indexes fixed joystick positions like `axes[4]`, `buttons[7]`, `buttons[5]`, etc., without validating array lengths. Malformed or shorter `Joy` messages can throw exceptions and trigger damping/stop behavior.

4. `joystick.py` maps axes and buttons by evdev capability enumeration order, while `loco_client.py` assumes a fixed semantic layout. Different controllers or kernel mappings can make all controls wrong.

5. `joystick.py` returns early if no joystick is found, leaving the node alive but without its normal timer, lock, axes, or buttons initialized. It logs an error but does not fail fast.

6. `loco_client.entering_balancing()` sets `self.balanced = True` even if the balancing loop exited because of a problem or because `robot_stopped` became true.

7. `loco_client.entering_balancing()` calls `SetStandHeight()` in a tight loop without any sleep or rate control, potentially hammering the robot control interface.

8. `loco_client.entering_balancing()` calls `BalanceStand`, `SetStandHeight`, and `Start` even after logging “Problems during balancing.”

9. `loco_client.py` publishes arm enable/home commands, but no node consumes them, so the joystick arm mode/home buttons do not affect OpenSoT.

10. `loco_client.py` publishes gripper `String` commands to topics the DX3 controller does not subscribe to.

---

## Manipulation / OpenSoT bugs

1. `opensot_solver.py` imports `pyopensot_collision` and `CollisionAvoidance` unconditionally. If collision dependencies are missing, the node cannot start even with `enable_collision_avoidance:=false`.

2. `enable_collision_avoidance` is declared and passed by launch, but ignored. Collision avoidance is always constructed and always added to the stack.

3. `make_6dof_marker()` in `opensot_solver.py` duplicates the menu-control insertion and server insertion block. The same interactive marker is inserted twice and receives two menu controls.

4. `opensot_solver.initialize()` ignores current robot joint positions when initializing the OpenSoT model. It reads `self.all_motor_q` earlier but then sets `self.q[7:] = q_init`. The first active command can jump the arms/waist toward `q_init`.

5. The control loop likely integrates velocity incorrectly: `dq = solver.solve()` is assigned directly via `self.q = self.model.sum(self.q, dq)` without multiplying by `control_dt`. If `solve()` returns joint velocities, this integrates far too fast.

6. The emergency-stop branch inside `control_loop()` is unreachable when `self.emergency_stop` is true because the outer condition is `if self.start_opensot and not self.emergency_stop:`. During emergency stop, the loop simply stops sending the damping/safe command branch.

7. In `use_robot=False`, `control_loop()` returns before `publishCollisionDistances()`, so collision debug markers are not published in simulation/visualization mode.

8. `opensot_solver.py` publishes `world -> pelvis`, while `fix_mola_odometry.py` publishes `map -> pelvis` and `robot_state_launcher` publishes `base_link -> pelvis`. These create conflicting TF parents for `pelvis`.

9. `opensot_solver.py` blocks indefinitely waiting for `/robot_state_publisher/get_parameters`. There is no timeout or fallback.

10. `opensot_solver.py` creates a static-ish `world -> pelvis` transform once in the constructor and then later broadcasts dynamic `world -> pelvis`. This can cause confusing TF behavior at startup.

11. `opensot_solver.py` uses hard-coded task frames `right_hand_point_contact` and `left_hand_point_contact`. Some URDF variants do not contain these frames.

12. `manipulation_launcher.launch.py` passes `arm_controlled` only to `dx3_controller`, not to `opensot_solver`, so OpenSoT does not respect left/right/both arm selection.

13. `dx3_hand.py` has no `use_robot` or `send_commands` launch parameter support. It always initializes the Unitree channel layer and `self.send_commands` is hard-coded to `True`.

14. `dx3_hand.py` publishes motor states on relative topics `g1pilot/dx3/left/motor_state` and `g1pilot/dx3/right/motor_state`, unlike the rest of the repo’s absolute `/g1pilot/...` convention. Under a namespace, these topics will move unexpectedly.

---

## Robot state and TF bugs

1. `robot_state.py` publishes `JointState.name` with 29 names, but `position` can be shorter if `msg.motor_state` has fewer entries. That produces invalid `JointState` messages with mismatched array lengths.

2. `robot_state.py` receives `sim_rate_hz` from launch but ignores it. Simulation mode always runs at hard-coded `0.05` seconds, i.e. 20 Hz.

3. In simulation mode, `robot_state.py` publishes IMU and joint states but does not publish `/g1pilot/motor_state`.

4. `robot_state.py` publishes IMU messages with `frame_id = "pelvis"` while also publishing a dynamic `pelvis -> imu_link` transform. If `imu_link` exists, the IMU message should normally use that sensor frame.

5. The `pelvis -> imu_link` transform uses the live IMU orientation as the transform rotation. A sensor mount transform should be fixed; this likely corrupts the TF tree by treating attitude as a sensor mounting offset.

6. `robot_state_launcher.launch.py` publishes a static `base_link -> pelvis` transform, but the default URDF root is `pelvis` and does not define `base_link`.

7. The node is named `pelvis_to_base_link_tf`, but the arguments publish parent `base_link`, child `pelvis`, the opposite of the name.

8. `robot_state_launcher.launch.py` publishes `mid360_link -> livox_frame` even though `g1_29dof.urdf` already defines a fixed `livox_joint` from `mid360_link` to `livox_frame`. That duplicates the same child frame.

9. `fix_mola_odometry.py` logs that it applies `Rz(π)`, but the code sets `q_fix = euler_to_quat(0,0,0)` and then only negates quaternion `w`. That does not implement the logged 180-degree yaw correction.

10. `fix_mola_odometry.py` flips Y position but copies twist and covariance unchanged. The twist/covariance no longer match the transformed pose.

11. `fix_mola_odometry.py` publishes `map -> pelvis`, conflicting with other `pelvis` TF publishers.

---

## URDF/model bugs

1. `description_files/urdf/g1_29dof_dx3.urdf` defines `pelvis_contour_joint` twice. URDF joint names must be unique.

2. `description_files/urdf/g1_29dof_dx3_upperbody.urdf` contains absolute mesh paths under `/home/cdonoso/Desktop/ROS/g1pilot/...`, so it is not portable and will fail on other machines.

3. `g1_29dof_dx3.urdf` is missing `left_hand_point_contact` and `right_hand_point_contact`.

4. `g1_29dof_dx3_upperbody.urdf` is also missing `left_hand_point_contact` and `right_hand_point_contact`.

5. The DX3 URDF variants are missing `livox_frame`, but launch files assume `mid360_link` / `livox_frame` exist.

6. The default launch always uses `g1_29dof.urdf`; the `config.yaml` robot model setting is not wired into launch selection.

---

## Docker and scripts bugs

1. `docker/docker-compose.yml` references `env.conf`, but no `docker/env.conf` exists in the repository.

2. The compose healthcheck waits for `/base_height`. That topic is only published by OpenSoT while OpenSoT is active, so the container can be marked unhealthy during normal startup.

3. `docker/docker-compose.yml` uses `retries: 10e9`, which is not a normal integer value for Compose schema purposes.

4. `docker/Dockerfile` is based on ROS Jazzy, but line 369 builds with `-DHUMBLE_ROS=humble`.

5. The repo mixes ROS distributions: README badge says Jazzy, README text says Ubuntu 22.04 + Humble, main Dockerfile uses Jazzy, and `camera.Dockerfile` uses Humble.

6. `docker/Dockerfile:29` uses an SSH Git URL, `git@github.com:advrhumanoids/multidof_recipes.git`. Clean Docker builds usually do not have SSH keys or known hosts configured.

7. `docker/camera.Dockerfile` runs `sysctl` during image build. Kernel sysctls should be runtime/container/host configuration, not Docker build steps.

8. `docker/cbuild` defaults to skipping and cleaning `livox_ros_driver2`. The entrypoint runs `./cbuild`, so the Livox package may be removed from the install space unless `BUILD_LIVOX=1`.

9. README commands say `sh build.sh`, `sh run.sh`, etc., but those scripts are inside `docker/`. Running them from the repository root will fail unless the user first changes directory or prefixes `docker/`.

10. `docker/setup_uri.sh` has comments before the shebang. If executed directly, the shebang is not the first line.

11. `docker/entrypoint.sh` refuses to run without `G1_INTERFACE`, so the provided Docker entrypoint cannot run offline/simulation bringup.

12. `docker/run.sh` and `docker/run_camera.sh` assume they are executed from inside the `docker` directory because they mount `$(pwd)/../`.

---

## Documentation/config bugs

1. `docs/INSTRUCTIONS.md` is empty, but the README links to it for more details.

2. README says there is an `arm_controller` node, but the actual entry point is `opensot_solver`.

3. README says the package provides MOLA odometry and path planner integration, but default bringup does not start MOLA, Livox, or the Dijkstra planner.

4. `docs/CHEATS.md` uses the wrong goal message type for `/g1pilot/goal`.

5. `docs/CHEATS.md` uses DX3 hand topics/types that do not match `dx3_hand.py`.

6. `docs/CHEATS.md` documents `/g1pilot/arms/enabled` and `/g1pilot/arms/home`, but no subscriber implements those commands.

7. `config/config.yaml` contains many parameters and topics that are not read anywhere, so it gives users the impression they can configure behavior that is actually hard-coded.

8. `config/config.yaml` says `navigation.autonomous_topic: /g1pilot/joy_autonomous`, but the code uses `/g1pilot/auto_joy`.

9. `config/config.yaml` says `robot.left_eff_topic: /g1pilot/left_hand_goal` and `robot.right_eff_topic: /g1pilot/right_hand_goal`, but the code uses other topic names.

10. RViz configs are configured around topics that do not match the active planner/OpenSoT subscribers.

The highest-priority fixes are to unify the topic/message contract, fix the launch composition so bringup actually starts planner/Livox/MOLA as intended, correct the `nav2point`/`loco_client` button mismatch, make simulation/offline launches not require `G1_INTERFACE`, and repair packaging of RViz/config/pipeline assets.

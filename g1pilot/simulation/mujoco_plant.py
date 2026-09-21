#!/usr/bin/env python3

import argparse
import collections
import importlib
import os
import threading
import time
from copy import deepcopy
from pathlib import Path

import mujoco
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__HandCmd_,
    unitree_hg_msg_dds__HandState_,
    unitree_go_msg_dds__WirelessController_,
    unitree_hg_msg_dds__IMUState_,
    unitree_hg_msg_dds__LowCmd_,
    unitree_hg_msg_dds__LowState_,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, IMUState_, LowCmd_, LowState_

from g1pilot.simulation.lowcmd_utils import (
    ARM_MOTOR_IDS,
    LEG_MOTOR_IDS,
    OPENHOMIE_OBS_MOTOR_IDS,
    WAIST_YAW_MOTOR_ID,
    clamp,
)
from g1pilot.simulation.openhomie_policy import (
    OpenHomiePolicy,
    compute_openhomie_observation,
    openhomie_single_obs_dim,
)


G1_NUM_BODY_MOTORS = 29
G1_LOCKED_WAIST_MOTOR_IDS = (13, 14)
DEX3_NUM_MOTORS = 7
OPENHOMIE_DEFAULT_KPS = np.array(
    [100.0, 100.0, 100.0, 150.0, 40.0, 40.0, 100.0, 100.0, 100.0, 150.0, 40.0, 40.0],
    dtype=np.float64,
)
OPENHOMIE_DEFAULT_KDS = np.array(
    [2.0, 2.0, 2.0, 4.0, 2.0, 2.0, 2.0, 2.0, 2.0, 4.0, 2.0, 2.0],
    dtype=np.float64,
)
OPENHOMIE_DEFAULT_ANGLES = np.array(
    [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0],
    dtype=np.float64,
)
OPENHOMIE_CMD_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)
UPPER_BODY_MOTOR_IDS = (WAIST_YAW_MOTOR_ID,) + ARM_MOTOR_IDS
UPPER_BODY_NOMINAL_Q = np.zeros(len(UPPER_BODY_MOTOR_IDS), dtype=np.float64)
UPPER_BODY_HOLD_KP = 100.0
UPPER_BODY_HOLD_KD = 0.5
ACTUATOR_MOTOR_IDS = {
    "left_hip_pitch": 0,
    "left_hip_roll": 1,
    "left_hip_yaw": 2,
    "left_knee": 3,
    "left_ankle_pitch": 4,
    "left_ankle_roll": 5,
    "right_hip_pitch": 6,
    "right_hip_roll": 7,
    "right_hip_yaw": 8,
    "right_knee": 9,
    "right_ankle_pitch": 10,
    "right_ankle_roll": 11,
    "waist_yaw": 12,
    "waist_roll": 13,
    "waist_pitch": 14,
    "left_shoulder_pitch": 15,
    "left_shoulder_roll": 16,
    "left_shoulder_yaw": 17,
    "left_elbow": 18,
    "left_wrist_roll": 19,
    "left_wrist_pitch": 20,
    "left_wrist_yaw": 21,
    "right_shoulder_pitch": 22,
    "right_shoulder_roll": 23,
    "right_shoulder_yaw": 24,
    "right_elbow": 25,
    "right_wrist_roll": 26,
    "right_wrist_pitch": 27,
    "right_wrist_yaw": 28,
}
DEX3_JOINT_ORDER = (
    "thumb_0",
    "thumb_1",
    "thumb_2",
    "index_0",
    "index_1",
    "middle_0",
    "middle_1",
)
LEFT_HAND_ACTUATOR_IDS = {
    f"left_hand_{name}_joint": motor_id for motor_id, name in enumerate(DEX3_JOINT_ORDER)
}
RIGHT_HAND_ACTUATOR_IDS = {
    f"right_hand_{name}_joint": motor_id for motor_id, name in enumerate(DEX3_JOINT_ORDER)
}


def make_mode(motor_id, status, timeout):
    return (motor_id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7)


def make_default_hand_cmd():
    cmd = unitree_hg_msg_dds__HandCmd_()
    if len(cmd.motor_cmd) != DEX3_NUM_MOTORS:
        raise RuntimeError(
            f"Unitree HandCmd_ motor_cmd has length {len(cmd.motor_cmd)}, expected {DEX3_NUM_MOTORS}"
        )
    for i in range(DEX3_NUM_MOTORS):
        motor_cmd = cmd.motor_cmd[i]
        motor_cmd.mode = make_mode(i, status=0x01, timeout=0)
        motor_cmd.q = 0.0
        motor_cmd.dq = 0.0
        motor_cmd.kp = 1.5
        motor_cmd.kd = 0.1
        motor_cmd.tau = 0.0
    return cmd


def repo_root():
    return Path(__file__).resolve().parents[2]


def default_xml():
    return repo_root() / "description_files" / "xml" / "openhomie_g1_29dof.xml"


def default_policy_path():
    env_path = os.environ.get("OPENHOMIE_POLICY_PATH", "").strip()
    if env_path:
        return env_path
    candidate = repo_root().parent / "reference_repos" / "OpenHomie" / "HomieDeploy" / "deploy.onnx"
    return str(candidate) if candidate.exists() else ""


def make_body_command():
    return {
        "q": np.zeros(G1_NUM_BODY_MOTORS, dtype=np.float64),
        "dq": np.zeros(G1_NUM_BODY_MOTORS, dtype=np.float64),
        "kp": np.zeros(G1_NUM_BODY_MOTORS, dtype=np.float64),
        "kd": np.zeros(G1_NUM_BODY_MOTORS, dtype=np.float64),
        "tau": np.zeros(G1_NUM_BODY_MOTORS, dtype=np.float64),
    }


class OpenHomiePlantPolicy:
    """OpenHomie lower-body policy owned by the MuJoCo plant episode."""

    def __init__(self, args):
        self.policy_path = str(args.policy_path).strip()
        self.allow_zero_policy = bool(args.allow_zero_policy)
        self.mode = str(args.mode).strip().lower()
        self.control_decimation = int(args.control_decimation)
        self.obs_history_len = int(args.obs_history_len)
        self.expected_num_obs = int(args.expected_num_obs)
        self.height_cmd = float(args.height_cmd)
        self.action_scale = float(args.action_scale)
        self.ang_vel_scale = float(args.ang_vel_scale)
        self.dof_pos_scale = float(args.dof_pos_scale)
        self.dof_vel_scale = float(args.dof_vel_scale)
        self.max_vx = float(args.max_vx)
        self.max_vy = float(args.max_vy)
        self.max_yaw_rate = float(args.max_yaw_rate)
        self.command = np.array(
            [
                clamp(args.vx, -self.max_vx, self.max_vx),
                clamp(args.vy, -self.max_vy, self.max_vy),
                clamp(args.yaw_rate, -self.max_yaw_rate, self.max_yaw_rate),
            ],
            dtype=np.float32,
        )

        if self.mode not in ("stand", "walk"):
            raise RuntimeError(f"--mode must be 'stand' or 'walk', got {self.mode!r}")
        if self.control_decimation <= 0:
            raise RuntimeError("--control-decimation must be positive")

        self.leg_motor_ids = tuple(LEG_MOTOR_IDS)
        self.obs_motor_ids = tuple(OPENHOMIE_OBS_MOTOR_IDS)
        self.kps = OPENHOMIE_DEFAULT_KPS.copy()
        self.kds = OPENHOMIE_DEFAULT_KDS.copy()
        self.default_angles = OPENHOMIE_DEFAULT_ANGLES.copy()
        self.cmd_scale = OPENHOMIE_CMD_SCALE.copy()

        self.single_obs_dim = openhomie_single_obs_dim(len(self.obs_motor_ids), len(self.leg_motor_ids))
        self.num_obs = self.single_obs_dim * self.obs_history_len
        if self.num_obs != self.expected_num_obs:
            raise RuntimeError(
                f"OpenHomie observation size mismatch: got {self.num_obs}, expected {self.expected_num_obs}"
            )

        self.policy = self._load_policy()
        self.last_action = np.zeros(len(self.leg_motor_ids), dtype=np.float32)
        self.target_dof_pos = self.default_angles.copy()
        self.obs_history = collections.deque(maxlen=self.obs_history_len)
        for _ in range(self.obs_history_len):
            self.obs_history.append(np.zeros(self.single_obs_dim, dtype=np.float32))
        self.physics_sample_count = 0
        self.reported_policy_start = False

    def _load_policy(self):
        if not self.policy_path:
            if self.allow_zero_policy:
                print("OpenHomie policy disabled: using zero-action policy.")
                return None
            raise RuntimeError("--policy-path is required unless --allow-zero-policy is set")
        policy = OpenHomiePolicy(self.policy_path, num_obs=self.num_obs, num_actions=len(self.leg_motor_ids))
        print(
            f"Loaded OpenHomie policy: {self.policy_path} "
            f"({policy.backend}, {policy.input_shape} -> {policy.output_shape})"
        )
        return policy

    def active_command(self):
        if self.mode == "stand":
            return np.zeros(3, dtype=np.float32)
        return self.command.copy()

    def fill_leg_command(self, body_cmd):
        for i, motor_id in enumerate(self.leg_motor_ids):
            body_cmd["q"][motor_id] = float(self.target_dof_pos[i])
            body_cmd["dq"][motor_id] = 0.0
            body_cmd["kp"][motor_id] = float(self.kps[i])
            body_cmd["kd"][motor_id] = float(self.kds[i])
            body_cmd["tau"][motor_id] = 0.0

    def _compute_single_observation(self, obs):
        qj = np.array([obs["body_q"][motor_id] for motor_id in self.obs_motor_ids], dtype=np.float32)
        dqj = np.array([obs["body_dq"][motor_id] for motor_id in self.obs_motor_ids], dtype=np.float32)
        return compute_openhomie_observation(
            command=self.active_command(),
            height_cmd=self.height_cmd,
            omega=np.asarray(obs["floating_base_vel"][3:6], dtype=np.float32),
            quat_wxyz=np.asarray(obs["floating_base_pose"][3:7], dtype=np.float32),
            qj=qj,
            dqj=dqj,
            last_action=self.last_action,
            default_angles=self.default_angles,
            cmd_scale=self.cmd_scale,
            ang_vel_scale=self.ang_vel_scale,
            dof_pos_scale=self.dof_pos_scale,
            dof_vel_scale=self.dof_vel_scale,
        )

    def update_after_step(self, obs):
        self.physics_sample_count += 1
        if self.physics_sample_count % self.control_decimation != 0:
            return

        single_obs = self._compute_single_observation(obs)
        self.obs_history.append(single_obs)
        policy_obs = np.concatenate(tuple(self.obs_history)).astype(np.float32)
        if self.policy is None:
            self.last_action = np.zeros(len(self.leg_motor_ids), dtype=np.float32)
        else:
            self.last_action = self.policy.run(policy_obs)
        self.target_dof_pos = self.last_action.astype(np.float64) * self.action_scale + self.default_angles

        if not self.reported_policy_start:
            action_text = ", ".join(f"{value:.4f}" for value in self.last_action)
            target_text = ", ".join(f"{value:.4f}" for value in self.target_dof_pos)
            print(
                f"Started OpenHomie policy updates after {self.physics_sample_count} physics steps "
                f"with control_decimation={self.control_decimation}"
            )
            print(f"First OpenHomie policy action: [{action_text}]")
            print(f"First OpenHomie policy target: [{target_text}]")
            self.reported_policy_start = True


class G1PilotUnitreeBridge:
    """GR00T-style DDS bridge: no direct ownership of MuJoCo model/data."""

    def __init__(self, *, num_body_motor, num_hand_motor):
        self.num_body_motor = int(num_body_motor)
        self.num_hand_motor = int(num_hand_motor)

        self.arm_sdk_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.left_hand_cmd = make_default_hand_cmd()
        self.right_hand_cmd = make_default_hand_cmd()
        self.left_hand_state = unitree_hg_msg_dds__HandState_()
        self.right_hand_state = unitree_hg_msg_dds__HandState_()
        self.torso_imu_state = unitree_hg_msg_dds__IMUState_()
        self.wireless_controller = unitree_go_msg_dds__WirelessController_()

        self.arm_sdk_lock = threading.Lock()
        self.left_hand_cmd_lock = threading.Lock()
        self.right_hand_cmd_lock = threading.Lock()
        self.arm_sdk_received = False
        self.left_hand_cmd_received = False
        self.right_hand_cmd_received = False
        self.new_arm_sdk_cmd = False
        self.new_left_hand_cmd = False
        self.new_right_hand_cmd = False

        self.low_state_publisher = ChannelPublisher("rt/lowstate", LowState_)
        self.low_state_publisher.Init()
        self.left_hand_state_publisher = ChannelPublisher("rt/dex3/left/state", HandState_)
        self.left_hand_state_publisher.Init()
        self.right_hand_state_publisher = ChannelPublisher("rt/dex3/right/state", HandState_)
        self.right_hand_state_publisher.Init()
        self.torso_imu_publisher = ChannelPublisher("rt/secondary_imu", IMUState_)
        self.torso_imu_publisher.Init()
        self.wireless_controller_publisher = ChannelPublisher("rt/wirelesscontroller", WirelessController_)
        self.wireless_controller_publisher.Init()

        self.arm_sdk_subscriber = ChannelSubscriber("rt/arm_sdk", LowCmd_)
        self.arm_sdk_subscriber.Init(self.arm_sdk_handler, 1)
        self.left_hand_cmd_subscriber = ChannelSubscriber("rt/dex3/left/cmd", HandCmd_)
        self.left_hand_cmd_subscriber.Init(self.left_hand_cmd_handler, 1)
        self.right_hand_cmd_subscriber = ChannelSubscriber("rt/dex3/right/cmd", HandCmd_)
        self.right_hand_cmd_subscriber.Init(self.right_hand_cmd_handler, 1)

    def arm_sdk_handler(self, msg):
        with self.arm_sdk_lock:
            self.arm_sdk_cmd = deepcopy(msg)
            self.arm_sdk_received = True
            self.new_arm_sdk_cmd = True

    def left_hand_cmd_handler(self, msg):
        with self.left_hand_cmd_lock:
            self.left_hand_cmd = deepcopy(msg)
            self.left_hand_cmd_received = True
            self.new_left_hand_cmd = True

    def right_hand_cmd_handler(self, msg):
        with self.right_hand_cmd_lock:
            self.right_hand_cmd = deepcopy(msg)
            self.right_hand_cmd_received = True
            self.new_right_hand_cmd = True

    def get_arm_sdk_cmd(self):
        with self.arm_sdk_lock:
            return deepcopy(self.arm_sdk_cmd), self.arm_sdk_received

    def get_left_hand_cmd(self):
        with self.left_hand_cmd_lock:
            return deepcopy(self.left_hand_cmd), self.left_hand_cmd_received

    def get_right_hand_cmd(self):
        with self.right_hand_cmd_lock:
            return deepcopy(self.right_hand_cmd), self.right_hand_cmd_received

    def publish_lowstate(self, obs):
        for i in range(self.num_body_motor):
            self.low_state.motor_state[i].q = float(obs["body_q"][i])
            self.low_state.motor_state[i].dq = float(obs["body_dq"][i])
            self.low_state.motor_state[i].ddq = float(obs["body_ddq"][i])
            self.low_state.motor_state[i].tau_est = float(obs["body_tau_est"][i])

        self.low_state.imu_state.quaternion[:] = obs["floating_base_pose"][3:7]
        self.low_state.imu_state.gyroscope[:] = obs["floating_base_vel"][3:6]
        self.low_state.imu_state.accelerometer[:] = obs["floating_base_acc"][:3]

        self.torso_imu_state.quaternion[:] = obs["secondary_imu_quat"]
        self.torso_imu_state.gyroscope[:] = obs["secondary_imu_vel"][3:6]

        tick_ms = int(obs["time"] * 1e3)
        self.low_state.tick = tick_ms

        self.low_state_publisher.Write(self.low_state)
        self.torso_imu_publisher.Write(self.torso_imu_state)

        for i in range(self.num_hand_motor):
            self.left_hand_state.motor_state[i].q = float(obs["left_hand_q"][i])
            self.left_hand_state.motor_state[i].dq = float(obs["left_hand_dq"][i])
            self.right_hand_state.motor_state[i].q = float(obs["right_hand_q"][i])
            self.right_hand_state.motor_state[i].dq = float(obs["right_hand_dq"][i])
        self.left_hand_state_publisher.Write(self.left_hand_state)
        self.right_hand_state_publisher.Write(self.right_hand_state)

    def publish_wireless_controller(self):
        self.wireless_controller_publisher.Write(self.wireless_controller)


class G1PilotMujocoEnv:
    """GR00T-style MuJoCo env: owns model/data, prepares obs, computes torques, steps."""

    def __init__(self, *, xml_path, sim_dt, onscreen=False):
        self.xml_path = Path(xml_path).expanduser().resolve()
        if not self.xml_path.exists():
            raise RuntimeError(f"MuJoCo XML does not exist: {self.xml_path}")

        self.mj_model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.mj_data = mujoco.MjData(self.mj_model)
        self.mj_model.opt.timestep = float(sim_dt)
        self.sim_dt = float(sim_dt)
        self.num_actuators = self.mj_model.nu
        self.num_body_motors = G1_NUM_BODY_MOTORS
        self.num_hand_motors = DEX3_NUM_MOTORS

        self.qpos_offset = 7
        self.qvel_offset = 6
        self.body_actuator_indices = []
        self.body_motor_ids = []
        self.body_qpos_addr = []
        self.body_qvel_addr = []
        self.left_hand_actuator_indices = []
        self.left_hand_motor_ids = []
        self.left_hand_qpos_addr = []
        self.left_hand_qvel_addr = []
        self.right_hand_actuator_indices = []
        self.right_hand_motor_ids = []
        self.right_hand_qpos_addr = []
        self.right_hand_qvel_addr = []
        for i in range(self.num_actuators):
            actuator_name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            joint_id = int(self.mj_model.actuator_trnid[i, 0])
            qpos_addr = int(self.mj_model.jnt_qposadr[joint_id])
            qvel_addr = int(self.mj_model.jnt_dofadr[joint_id])
            if actuator_name in ACTUATOR_MOTOR_IDS:
                motor_id = ACTUATOR_MOTOR_IDS[actuator_name]
                self.body_actuator_indices.append(i)
                self.body_motor_ids.append(motor_id)
                self.body_qpos_addr.append(qpos_addr)
                self.body_qvel_addr.append(qvel_addr)
            elif actuator_name in LEFT_HAND_ACTUATOR_IDS:
                self.left_hand_actuator_indices.append(i)
                self.left_hand_motor_ids.append(LEFT_HAND_ACTUATOR_IDS[actuator_name])
                self.left_hand_qpos_addr.append(qpos_addr)
                self.left_hand_qvel_addr.append(qvel_addr)
            elif actuator_name in RIGHT_HAND_ACTUATOR_IDS:
                self.right_hand_actuator_indices.append(i)
                self.right_hand_motor_ids.append(RIGHT_HAND_ACTUATOR_IDS[actuator_name])
                self.right_hand_qpos_addr.append(qpos_addr)
                self.right_hand_qvel_addr.append(qvel_addr)
            else:
                raise RuntimeError(f"Unexpected MuJoCo actuator name: {actuator_name!r}")
        self.body_actuator_indices = np.asarray(self.body_actuator_indices, dtype=np.int32)
        self.body_motor_ids = np.asarray(self.body_motor_ids, dtype=np.int32)
        self.body_qpos_addr = np.asarray(self.body_qpos_addr, dtype=np.int32)
        self.body_qvel_addr = np.asarray(self.body_qvel_addr, dtype=np.int32)
        self.left_hand_actuator_indices = np.asarray(self.left_hand_actuator_indices, dtype=np.int32)
        self.left_hand_motor_ids = np.asarray(self.left_hand_motor_ids, dtype=np.int32)
        self.left_hand_qpos_addr = np.asarray(self.left_hand_qpos_addr, dtype=np.int32)
        self.left_hand_qvel_addr = np.asarray(self.left_hand_qvel_addr, dtype=np.int32)
        self.right_hand_actuator_indices = np.asarray(self.right_hand_actuator_indices, dtype=np.int32)
        self.right_hand_motor_ids = np.asarray(self.right_hand_motor_ids, dtype=np.int32)
        self.right_hand_qpos_addr = np.asarray(self.right_hand_qpos_addr, dtype=np.int32)
        self.right_hand_qvel_addr = np.asarray(self.right_hand_qvel_addr, dtype=np.int32)

        for locked_motor_id in G1_LOCKED_WAIST_MOTOR_IDS:
            if locked_motor_id in self.body_motor_ids:
                raise RuntimeError(
                    "Locked-waist MuJoCo XML must not expose waist roll/pitch actuators; "
                    f"found motor slot {locked_motor_id}."
                )
        expected_body_actuators = self.num_body_motors - len(G1_LOCKED_WAIST_MOTOR_IDS)
        if len(self.body_actuator_indices) != expected_body_actuators:
            raise RuntimeError(
                f"Expected {expected_body_actuators} active body actuators, "
                f"found {len(self.body_actuator_indices)}."
            )
        if len(self.left_hand_actuator_indices) != self.num_hand_motors:
            raise RuntimeError(
                f"Expected {self.num_hand_motors} left hand actuators, "
                f"found {len(self.left_hand_actuator_indices)}."
            )
        if len(self.right_hand_actuator_indices) != self.num_hand_motors:
            raise RuntimeError(
                f"Expected {self.num_hand_motors} right hand actuators, "
                f"found {len(self.right_hand_actuator_indices)}."
            )

        self.torso_index = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
        if self.torso_index < 0:
            raise RuntimeError("Expected body 'torso_link' in MuJoCo model")

        self.torques = np.zeros(self.num_actuators, dtype=np.float64)
        self.unitree_bridge = None
        self.viewer = None
        self.onscreen = bool(onscreen)
        mujoco.mj_forward(self.mj_model, self.mj_data)

    def set_unitree_bridge(self, bridge):
        self.unitree_bridge = bridge

    def launch_viewer(self):
        mujoco_viewer = importlib.import_module("mujoco.viewer")
        self.viewer = mujoco_viewer.launch_passive(
            self.mj_model,
            self.mj_data,
            show_left_ui=False,
            show_right_ui=False,
        )
        self.viewer.cam.azimuth = 120
        self.viewer.cam.elevation = -30
        self.viewer.cam.distance = 2.0
        self.viewer.cam.lookat = np.array([0.0, 0.0, 0.5])
        self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.viewer.cam.trackbodyid = self.mj_model.body("pelvis").id

    def prepare_obs(self):
        obs = {}
        obs["floating_base_pose"] = self.mj_data.qpos[:7].copy()
        obs["floating_base_vel"] = self.mj_data.qvel[:6].copy()
        obs["floating_base_acc"] = self.mj_data.qacc[:6].copy()
        obs["secondary_imu_quat"] = self.mj_data.xquat[self.torso_index].copy()

        pose = np.zeros(13, dtype=np.float64)
        mujoco.mj_objectVelocity(
            self.mj_model,
            self.mj_data,
            mujoco.mjtObj.mjOBJ_BODY,
            self.torso_index,
            pose[7:13],
            1,
        )
        pose[7:10], pose[10:13] = pose[10:13], pose[7:10].copy()
        obs["secondary_imu_vel"] = pose[7:13]

        body_q = np.zeros(self.num_body_motors, dtype=np.float64)
        body_dq = np.zeros(self.num_body_motors, dtype=np.float64)
        body_ddq = np.zeros(self.num_body_motors, dtype=np.float64)
        body_tau_est = np.zeros(self.num_body_motors, dtype=np.float64)
        body_q[self.body_motor_ids] = self.mj_data.qpos[self.body_qpos_addr]
        body_dq[self.body_motor_ids] = self.mj_data.qvel[self.body_qvel_addr]
        body_ddq[self.body_motor_ids] = self.mj_data.qacc[self.body_qvel_addr]
        body_tau_est[self.body_motor_ids] = self.mj_data.actuator_force[self.body_actuator_indices]
        obs["body_q"] = body_q
        obs["body_dq"] = body_dq
        obs["body_ddq"] = body_ddq
        obs["body_tau_est"] = body_tau_est

        left_hand_q = np.zeros(self.num_hand_motors, dtype=np.float64)
        left_hand_dq = np.zeros(self.num_hand_motors, dtype=np.float64)
        left_hand_q[self.left_hand_motor_ids] = self.mj_data.qpos[self.left_hand_qpos_addr]
        left_hand_dq[self.left_hand_motor_ids] = self.mj_data.qvel[self.left_hand_qvel_addr]
        right_hand_q = np.zeros(self.num_hand_motors, dtype=np.float64)
        right_hand_dq = np.zeros(self.num_hand_motors, dtype=np.float64)
        right_hand_q[self.right_hand_motor_ids] = self.mj_data.qpos[self.right_hand_qpos_addr]
        right_hand_dq[self.right_hand_motor_ids] = self.mj_data.qvel[self.right_hand_qvel_addr]
        obs["left_hand_q"] = left_hand_q
        obs["left_hand_dq"] = left_hand_dq
        obs["right_hand_q"] = right_hand_q
        obs["right_hand_dq"] = right_hand_dq
        obs["time"] = float(self.mj_data.time)
        return obs

    def compute_body_torques(self, body_cmd):
        body_torques = np.zeros(self.num_actuators, dtype=np.float64)

        q = self.mj_data.qpos[self.body_qpos_addr]
        dq = self.mj_data.qvel[self.body_qvel_addr]
        for local_id, (actuator_id, motor_id) in enumerate(zip(self.body_actuator_indices, self.body_motor_ids)):
            motor_id = int(motor_id)
            body_torques[actuator_id] = (
                body_cmd["tau"][motor_id]
                + body_cmd["kp"][motor_id] * (body_cmd["q"][motor_id] - q[local_id])
                + body_cmd["kd"][motor_id] * (body_cmd["dq"][motor_id] - dq[local_id])
            )
        return body_torques

    def compute_hand_torques(self):
        left_cmd, _ = self.unitree_bridge.get_left_hand_cmd()
        right_cmd, _ = self.unitree_bridge.get_right_hand_cmd()
        hand_torques = np.zeros(self.num_actuators, dtype=np.float64)

        left_q = self.mj_data.qpos[self.left_hand_qpos_addr]
        left_dq = self.mj_data.qvel[self.left_hand_qvel_addr]
        for local_id, actuator_id in enumerate(self.left_hand_actuator_indices):
            motor_id = int(self.left_hand_motor_ids[local_id])
            motor_cmd = left_cmd.motor_cmd[motor_id]
            hand_torques[actuator_id] = (
                motor_cmd.tau
                + motor_cmd.kp * (motor_cmd.q - left_q[local_id])
                + motor_cmd.kd * (motor_cmd.dq - left_dq[local_id])
            )

        right_q = self.mj_data.qpos[self.right_hand_qpos_addr]
        right_dq = self.mj_data.qvel[self.right_hand_qvel_addr]
        for local_id, actuator_id in enumerate(self.right_hand_actuator_indices):
            motor_id = int(self.right_hand_motor_ids[local_id])
            motor_cmd = right_cmd.motor_cmd[motor_id]
            hand_torques[actuator_id] = (
                motor_cmd.tau
                + motor_cmd.kp * (motor_cmd.q - right_q[local_id])
                + motor_cmd.kd * (motor_cmd.dq - right_dq[local_id])
            )
        return hand_torques

    def sim_step(self, body_cmd):
        self.torques[:] = self.compute_body_torques(body_cmd)
        self.torques[:] += self.compute_hand_torques()
        self.mj_data.ctrl[:] = self.torques
        mujoco.mj_step(self.mj_model, self.mj_data)

    def update_viewer(self):
        if self.viewer is not None:
            self.viewer.sync()

    def viewer_running(self):
        return self.viewer is None or self.viewer.is_running()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()


class G1PilotMujocoPlant:
    def __init__(self, args):
        self.args = args
        self.env = G1PilotMujocoEnv(xml_path=args.xml, sim_dt=args.sim_dt, onscreen=not args.headless)
        self.bridge = G1PilotUnitreeBridge(
            num_body_motor=self.env.num_body_motors,
            num_hand_motor=self.env.num_hand_motors,
        )
        self.env.set_unitree_bridge(self.bridge)
        self.base_policy = OpenHomiePlantPolicy(args)
        self._reported_arm_sdk = False
        self._reported_no_arm_sdk = False
        if not args.headless:
            self.env.launch_viewer()

    def publish_current_state(self):
        obs = self.env.prepare_obs()
        self.bridge.publish_lowstate(obs)
        self.bridge.publish_wireless_controller()
        return obs

    def build_body_command(self):
        body_cmd = make_body_command()
        self.base_policy.fill_leg_command(body_cmd)
        self._fill_upper_body_command(body_cmd)
        return body_cmd

    def _fill_upper_body_command(self, body_cmd):
        arm_sdk_cmd, arm_sdk_received = self.bridge.get_arm_sdk_cmd()
        if arm_sdk_received:
            if not self._reported_arm_sdk:
                print("Received first rt/arm_sdk command; upper body now follows OpenSoT intent.")
                self._reported_arm_sdk = True
            for motor_id in UPPER_BODY_MOTOR_IDS:
                motor_cmd = arm_sdk_cmd.motor_cmd[int(motor_id)]
                body_cmd["q"][motor_id] = float(motor_cmd.q)
                body_cmd["dq"][motor_id] = float(motor_cmd.dq)
                body_cmd["kp"][motor_id] = float(motor_cmd.kp)
                body_cmd["kd"][motor_id] = float(motor_cmd.kd)
                body_cmd["tau"][motor_id] = float(motor_cmd.tau)
            return

        if not self._reported_no_arm_sdk:
            print("No rt/arm_sdk command yet; holding waist yaw and arms at nominal zero targets.")
            self._reported_no_arm_sdk = True
        for motor_id, q in zip(UPPER_BODY_MOTOR_IDS, UPPER_BODY_NOMINAL_Q):
            body_cmd["q"][motor_id] = float(q)
            body_cmd["dq"][motor_id] = 0.0
            body_cmd["kp"][motor_id] = UPPER_BODY_HOLD_KP
            body_cmd["kd"][motor_id] = UPPER_BODY_HOLD_KD
            body_cmd["tau"][motor_id] = 0.0

    def print_runtime_summary(self):
        q0 = ", ".join(f"{value:.3f}" for value in self.env.mj_data.qpos[:3])
        qvel_norm = float(np.linalg.norm(self.env.mj_data.qvel))
        ctrl_norm = float(np.linalg.norm(self.env.mj_data.ctrl))
        print(
            f"sim t={self.env.mj_data.time:.2f}s base_pos=[{q0}] "
            f"qvel_norm={qvel_norm:.3f} ctrl_norm={ctrl_norm:.3f}"
        )

    def run(self):
        print(f"Loading MuJoCo XML: {self.env.xml_path}")
        print(
            "Model shape: "
            f"nq={self.env.mj_model.nq}, nv={self.env.mj_model.nv}, nu={self.env.mj_model.nu}, "
            f"body_motor_slots={self.env.num_body_motors}, hand_motor_slots={self.env.num_hand_motors}, "
            f"dt={self.env.mj_model.opt.timestep}"
        )
        self.publish_current_state()

        start = time.monotonic()
        next_report = start + self.args.report_interval
        try:
            while self.env.viewer_running():
                if self.args.duration > 0.0 and time.monotonic() - start >= self.args.duration:
                    break

                step_start = time.perf_counter()
                body_cmd = self.build_body_command()
                self.env.sim_step(body_cmd)
                obs = self.publish_current_state()
                self.base_policy.update_after_step(obs)
                self.env.update_viewer()

                now = time.monotonic()
                if self.args.report_interval > 0.0 and now >= next_report:
                    self.print_runtime_summary()
                    next_report = now + self.args.report_interval

                sleep_s = self.env.sim_dt - (time.perf_counter() - step_start)
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
        finally:
            self.env.close()


def main():
    parser = argparse.ArgumentParser(description="Run the G1Pilot MuJoCo plant using GR00T-style DDS/MuJoCo ownership.")
    parser.add_argument("--xml", type=Path, default=default_xml())
    parser.add_argument("--domain-id", type=int, default=1)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--sim-dt", type=float, default=0.002)
    parser.add_argument("--policy-path", default=default_policy_path())
    parser.add_argument("--allow-zero-policy", action="store_true")
    parser.add_argument("--mode", choices=("stand", "walk"), default="stand")
    parser.add_argument("--control-decimation", type=int, default=10)
    parser.add_argument("--obs-history-len", type=int, default=6)
    parser.add_argument("--expected-num-obs", type=int, default=456)
    parser.add_argument("--height-cmd", type=float, default=0.74)
    parser.add_argument("--action-scale", type=float, default=0.25)
    parser.add_argument("--ang-vel-scale", type=float, default=0.25)
    parser.add_argument("--dof-pos-scale", type=float, default=1.0)
    parser.add_argument("--dof-vel-scale", type=float, default=0.05)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--max-vx", type=float, default=0.3)
    parser.add_argument("--max-vy", type=float, default=0.2)
    parser.add_argument("--max-yaw-rate", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run; 0 means until viewer closes.")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--report-interval", type=float, default=1.0)
    args = parser.parse_args()

    print(f"Starting Unitree SDK2 DDS: domain_id={args.domain_id}, interface={args.interface!r}")
    ChannelFactoryInitialize(args.domain_id, args.interface)
    G1PilotMujocoPlant(args).run()


if __name__ == "__main__":
    main()

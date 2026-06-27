#!/usr/bin/env python3

import argparse
import importlib
import threading
import time
from copy import deepcopy
from pathlib import Path

import mujoco
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import (
    unitree_go_msg_dds__WirelessController_,
    unitree_hg_msg_dds__IMUState_,
    unitree_hg_msg_dds__LowCmd_,
    unitree_hg_msg_dds__LowState_,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_, LowCmd_, LowState_


G1_NUM_BODY_MOTORS = 29
G1_LOCKED_WAIST_MOTOR_IDS = (13, 14)
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


def repo_root():
    return Path(__file__).resolve().parents[2]


def default_xml():
    return repo_root() / "description_files" / "xml" / "openhomie_g1_29dof.xml"


class G1PilotUnitreeBridge:
    """GR00T-style DDS bridge: no direct ownership of MuJoCo model/data."""

    def __init__(self, *, num_body_motor):
        self.num_body_motor = int(num_body_motor)

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.torso_imu_state = unitree_hg_msg_dds__IMUState_()
        self.wireless_controller = unitree_go_msg_dds__WirelessController_()

        self.low_cmd_lock = threading.Lock()
        self.low_cmd_received = False
        self.new_low_cmd = False

        self.low_state_publisher = ChannelPublisher("rt/lowstate", LowState_)
        self.low_state_publisher.Init()
        self.torso_imu_publisher = ChannelPublisher("rt/secondary_imu", IMUState_)
        self.torso_imu_publisher.Init()
        self.wireless_controller_publisher = ChannelPublisher("rt/wirelesscontroller", WirelessController_)
        self.wireless_controller_publisher.Init()

        self.low_cmd_subscriber = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self.low_cmd_subscriber.Init(self.low_cmd_handler, 1)

    def low_cmd_handler(self, msg):
        with self.low_cmd_lock:
            self.low_cmd = deepcopy(msg)
            self.low_cmd_received = True
            self.new_low_cmd = True

    def cmd_received(self):
        with self.low_cmd_lock:
            return self.low_cmd_received

    def get_low_cmd(self):
        with self.low_cmd_lock:
            return deepcopy(self.low_cmd), self.low_cmd_received

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

        self.qpos_offset = 7
        self.qvel_offset = 6
        self.actuator_motor_ids = []
        self.body_qpos_addr = []
        self.body_qvel_addr = []
        for i in range(self.num_actuators):
            actuator_name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if actuator_name not in ACTUATOR_MOTOR_IDS:
                raise RuntimeError(f"Unexpected MuJoCo actuator name: {actuator_name!r}")
            motor_id = ACTUATOR_MOTOR_IDS[actuator_name]
            self.actuator_motor_ids.append(motor_id)
            joint_id = int(self.mj_model.actuator_trnid[i, 0])
            self.body_qpos_addr.append(int(self.mj_model.jnt_qposadr[joint_id]))
            self.body_qvel_addr.append(int(self.mj_model.jnt_dofadr[joint_id]))
        self.actuator_motor_ids = np.asarray(self.actuator_motor_ids, dtype=np.int32)
        self.body_qpos_addr = np.asarray(self.body_qpos_addr, dtype=np.int32)
        self.body_qvel_addr = np.asarray(self.body_qvel_addr, dtype=np.int32)

        for locked_motor_id in G1_LOCKED_WAIST_MOTOR_IDS:
            if locked_motor_id in self.actuator_motor_ids:
                raise RuntimeError(
                    "Locked-waist MuJoCo XML must not expose waist roll/pitch actuators; "
                    f"found motor slot {locked_motor_id}."
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
        body_q[self.actuator_motor_ids] = self.mj_data.qpos[self.body_qpos_addr]
        body_dq[self.actuator_motor_ids] = self.mj_data.qvel[self.body_qvel_addr]
        body_ddq[self.actuator_motor_ids] = self.mj_data.qacc[self.body_qvel_addr]
        body_tau_est[self.actuator_motor_ids] = self.mj_data.actuator_force[: self.num_actuators]
        obs["body_q"] = body_q
        obs["body_dq"] = body_dq
        obs["body_ddq"] = body_ddq
        obs["body_tau_est"] = body_tau_est
        obs["time"] = float(self.mj_data.time)
        return obs

    def compute_body_torques(self):
        low_cmd, received = self.unitree_bridge.get_low_cmd()
        body_torques = np.zeros(self.num_actuators, dtype=np.float64)
        if not received:
            return body_torques

        q = self.mj_data.qpos[self.body_qpos_addr]
        dq = self.mj_data.qvel[self.body_qvel_addr]
        for actuator_id, motor_id in enumerate(self.actuator_motor_ids):
            motor_cmd = low_cmd.motor_cmd[int(motor_id)]
            body_torques[actuator_id] = (
                motor_cmd.tau
                + motor_cmd.kp * (motor_cmd.q - q[actuator_id])
                + motor_cmd.kd * (motor_cmd.dq - dq[actuator_id])
            )
        return body_torques

    def sim_step(self):
        obs = self.prepare_obs()
        self.unitree_bridge.publish_lowstate(obs)
        self.unitree_bridge.publish_wireless_controller()
        self.torques[:] = self.compute_body_torques()
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
        self.bridge = G1PilotUnitreeBridge(num_body_motor=self.env.num_body_motors)
        self.env.set_unitree_bridge(self.bridge)
        if not args.headless:
            self.env.launch_viewer()

    def wait_for_first_command(self):
        if not self.args.wait_for_command:
            return
        print(f"Waiting up to {self.args.wait_timeout:.1f}s for first rt/lowcmd before stepping physics...")
        start = time.monotonic()
        while not self.bridge.cmd_received():
            self.bridge.publish_lowstate(self.env.prepare_obs())
            self.env.update_viewer()
            if not self.env.viewer_running():
                return
            if time.monotonic() - start >= self.args.wait_timeout:
                raise RuntimeError("Timed out waiting for first rt/lowcmd. Start the controller launch first.")
            time.sleep(self.env.sim_dt)
        print("Received first rt/lowcmd; starting physics.")

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
            f"dds_motor_slots={self.env.num_body_motors}, dt={self.env.mj_model.opt.timestep}"
        )
        self.wait_for_first_command()

        start = time.monotonic()
        next_report = start + self.args.report_interval
        try:
            while self.env.viewer_running():
                if self.args.duration > 0.0 and time.monotonic() - start >= self.args.duration:
                    break

                step_start = time.perf_counter()
                self.env.sim_step()
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
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run; 0 means until viewer closes.")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--report-interval", type=float, default=1.0)
    parser.add_argument("--no-wait-for-command", dest="wait_for_command", action="store_false")
    parser.add_argument("--wait-timeout", type=float, default=10.0)
    parser.set_defaults(wait_for_command=True)
    args = parser.parse_args()

    print(f"Starting Unitree SDK2 DDS: domain_id={args.domain_id}, interface={args.interface!r}")
    ChannelFactoryInitialize(args.domain_id, args.interface)
    G1PilotMujocoPlant(args).run()


if __name__ == "__main__":
    main()

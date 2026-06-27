#!/usr/bin/env python3

import argparse
import collections
import importlib.util
from pathlib import Path
import sys

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from g1pilot.simulation.openhomie_policy import (
    OpenHomiePolicy,
    compute_openhomie_observation,
    openhomie_single_obs_dim,
    pd_control,
)
from g1pilot.simulation.lowcmd_utils import G1_NUM_MOTORS, OPENHOMIE_OBS_MOTOR_IDS


def default_openhomie_root():
    candidate = REPO_ROOT.parent / "reference_repos" / "OpenHomie"
    return candidate


def default_generated_xml():
    return REPO_ROOT / "description_files" / "xml" / "openhomie_g1_29dof.xml"


def resolve_openhomie_path(path_text, legged_gym_root):
    return Path(str(path_text).format(LEGGED_GYM_ROOT_DIR=str(legged_gym_root))).expanduser()


def load_config(config_path, legged_gym_root):
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    for key in ("xml_path", "policy_path"):
        if key in config:
            config[key] = resolve_openhomie_path(config[key], legged_gym_root)

    for key in ("kps", "kds", "default_angles", "cmd_scale", "cmd_init"):
        config[key] = np.asarray(config[key], dtype=np.float32)

    return config


def load_openhomie_reference_module(openhomie_root):
    legged_gym_python = openhomie_root / "HomieRL" / "legged_gym"
    module_path = openhomie_root / "MujocoDeploy" / "mujoco_deploy_g1.py"
    if str(legged_gym_python) not in sys.path:
        sys.path.insert(0, str(legged_gym_python))
    spec = importlib.util.spec_from_file_location("openhomie_reference_mujoco_deploy_g1", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not import OpenHomie reference module: {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def compute_observation(data, config, action, cmd, height_cmd, n_joints):
    return compute_openhomie_observation(
        command=cmd,
        height_cmd=height_cmd,
        omega=data.qvel[3:6].copy(),
        quat_wxyz=data.qpos[3:7].copy(),
        qj=data.qpos[7 : 7 + n_joints].copy(),
        dqj=data.qvel[6 : 6 + n_joints].copy(),
        last_action=action,
        default_angles=config["default_angles"],
        cmd_scale=config["cmd_scale"],
        ang_vel_scale=config["ang_vel_scale"],
        dof_pos_scale=config["dof_pos_scale"],
        dof_vel_scale=config["dof_vel_scale"],
    )


def unitree_29_from_openhomie_27(values_27):
    values_27 = np.asarray(values_27, dtype=np.float32)
    if len(values_27) != 27:
        raise ValueError(f"Expected 27 OpenHomie active-joint values, got {len(values_27)}")
    values_29 = np.zeros(G1_NUM_MOTORS, dtype=np.float32)
    values_29[:13] = values_27[:13]
    values_29[15:29] = values_27[13:27]
    return values_29


def compute_observation_via_unitree_29_mapping(data, config, action, cmd, height_cmd, n_joints):
    q27 = data.qpos[7 : 7 + n_joints].copy()
    dq27 = data.qvel[6 : 6 + n_joints].copy()
    q29 = unitree_29_from_openhomie_27(q27)
    dq29 = unitree_29_from_openhomie_27(dq27)
    qj = np.array([q29[motor_id] for motor_id in OPENHOMIE_OBS_MOTOR_IDS], dtype=np.float32)
    dqj = np.array([dq29[motor_id] for motor_id in OPENHOMIE_OBS_MOTOR_IDS], dtype=np.float32)
    return compute_openhomie_observation(
        command=cmd,
        height_cmd=height_cmd,
        omega=data.qvel[3:6].copy(),
        quat_wxyz=data.qpos[3:7].copy(),
        qj=qj,
        dqj=dqj,
        last_action=action,
        default_angles=config["default_angles"],
        cmd_scale=config["cmd_scale"],
        ang_vel_scale=config["ang_vel_scale"],
        dof_pos_scale=config["dof_pos_scale"],
        dof_vel_scale=config["dof_vel_scale"],
    )


def copy_openhomie_state_to_generated_29(source_data, target_data):
    q27 = source_data.qpos[7:34].copy()
    dq27 = source_data.qvel[6:33].copy()
    q29 = unitree_29_from_openhomie_27(q27)
    dq29 = unitree_29_from_openhomie_27(dq27)

    target_data.qpos[:] = 0.0
    target_data.qvel[:] = 0.0
    target_data.qpos[:7] = source_data.qpos[:7]
    target_data.qvel[:6] = source_data.qvel[:6]
    target_data.qpos[7:36] = q29[:29]
    target_data.qvel[6:35] = dq29[:29]


def compute_observation_from_generated_29_sensors(data, config, action, cmd, height_cmd):
    motor_count = 29
    q29 = data.sensordata[:motor_count].copy()
    dq29 = data.sensordata[motor_count : 2 * motor_count].copy()
    qj = np.array([q29[motor_id] for motor_id in OPENHOMIE_OBS_MOTOR_IDS], dtype=np.float32)
    dqj = np.array([dq29[motor_id] for motor_id in OPENHOMIE_OBS_MOTOR_IDS], dtype=np.float32)
    sensor_offset = 3 * motor_count
    return compute_openhomie_observation(
        command=cmd,
        height_cmd=height_cmd,
        omega=data.sensordata[sensor_offset + 4 : sensor_offset + 7].copy(),
        quat_wxyz=data.sensordata[sensor_offset : sensor_offset + 4].copy(),
        qj=qj,
        dqj=dqj,
        last_action=action,
        default_angles=config["default_angles"],
        cmd_scale=config["cmd_scale"],
        ang_vel_scale=config["ang_vel_scale"],
        dof_pos_scale=config["dof_pos_scale"],
        dof_vel_scale=config["dof_vel_scale"],
    )


def format_vec(values, limit=12):
    arr = np.asarray(values).reshape(-1)
    shown = ", ".join(f"{v:.4f}" for v in arr[:limit])
    if len(arr) > limit:
        shown += ", ..."
    return f"[{shown}]"


def build_arg_parser():
    openhomie_root = default_openhomie_root()
    parser = argparse.ArgumentParser(
        description="Run OpenHomie's original G1 MuJoCo model headlessly and print policy I/O diagnostics."
    )
    parser.add_argument("--openhomie-root", type=Path, default=openhomie_root)
    parser.add_argument("--config", type=Path, default=None, help="Defaults to OpenHomie/MujocoDeploy/g1.yaml")
    parser.add_argument("--xml-path", type=Path, default=None, help="Defaults to xml_path from g1.yaml")
    parser.add_argument(
        "--policy-path",
        type=Path,
        default=None,
        help="Defaults to policy_path from g1.yaml if it exists, else HomieDeploy/deploy.onnx if present.",
    )
    parser.add_argument("--no-policy", action="store_true", help="Skip policy inference and hold zero actions.")
    parser.add_argument("--steps", type=int, default=500, help="Number of MuJoCo simulation steps to run.")
    parser.add_argument("--height-cmd", type=float, default=None, help="Override height_cmd from g1.yaml.")
    parser.add_argument("--vx", type=float, default=None)
    parser.add_argument("--vy", type=float, default=None)
    parser.add_argument("--yaw-rate", type=float, default=None)
    parser.add_argument(
        "--compare-reference",
        action="store_true",
        help="Compare shared G1Pilot observation math against OpenHomie's original compute_observation().",
    )
    parser.add_argument(
        "--compare-unitree-29-slot",
        action="store_true",
        help="Compare original 27-joint observation against simulated 29-slot Unitree motor ID selection.",
    )
    parser.add_argument(
        "--compare-generated-xml",
        action="store_true",
        help="Map frozen OpenHomie states into the generated 29-slot XML and compare sensor-derived observations.",
    )
    parser.add_argument("--generated-xml", type=Path, default=default_generated_xml())
    parser.add_argument("--compare-atol", type=float, default=1e-7)
    return parser


def main():
    args = build_arg_parser().parse_args()
    import mujoco

    openhomie_root = args.openhomie_root.expanduser().resolve()
    legged_gym_root = openhomie_root / "HomieRL" / "legged_gym"
    config_path = args.config or (openhomie_root / "MujocoDeploy" / "g1.yaml")
    config = load_config(config_path, legged_gym_root)

    xml_path = args.xml_path.expanduser().resolve() if args.xml_path else config["xml_path"]
    if not xml_path.exists():
        raise RuntimeError(f"MuJoCo XML not found: {xml_path}")

    policy_path = args.policy_path.expanduser().resolve() if args.policy_path else config["policy_path"]
    if not policy_path.exists():
        onnx_fallback = openhomie_root / "HomieDeploy" / "deploy.onnx"
        policy_path = onnx_fallback if onnx_fallback.exists() else policy_path

    height_cmd = float(config["height_cmd"] if args.height_cmd is None else args.height_cmd)
    cmd = config["cmd_init"].copy()
    if args.vx is not None:
        cmd[0] = args.vx
    if args.vy is not None:
        cmd[1] = args.vy
    if args.yaw_rate is not None:
        cmd[2] = args.yaw_rate

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    model.opt.timestep = float(config["simulation_dt"])
    reference_module = load_openhomie_reference_module(openhomie_root) if args.compare_reference else None
    generated_model = None
    generated_data = None
    if args.compare_generated_xml:
        generated_xml = args.generated_xml.expanduser().resolve()
        if not generated_xml.exists():
            raise RuntimeError(f"Generated XML does not exist: {generated_xml}")
        generated_model = mujoco.MjModel.from_xml_path(str(generated_xml))
        generated_data = mujoco.MjData(generated_model)
        if generated_model.nu != 29 or generated_model.nsensor != 95 or generated_model.nsensordata != 113:
            raise RuntimeError(
                "Generated XML does not have the expected Unitree bridge shape: "
                f"nu={generated_model.nu}, nsensor={generated_model.nsensor}, nsensordata={generated_model.nsensordata}"
            )

    n_joints = data.qpos.shape[0] - 7
    num_actions = int(config["num_actions"])
    single_obs_dim = openhomie_single_obs_dim(n_joints, num_actions)
    expected_num_obs = int(config["num_obs"])
    num_obs = single_obs_dim * int(config["obs_history_len"])
    if num_obs != expected_num_obs:
        raise RuntimeError(f"Observation size mismatch: got {num_obs}, expected {expected_num_obs}")

    print("OpenHomie MuJoCo baseline diagnostic")
    print(f"  openhomie_root: {openhomie_root}")
    print(f"  config: {config_path}")
    print(f"  xml: {xml_path}")
    print(f"  nq/nv/nu: {model.nq}/{model.nv}/{model.nu}")
    print(f"  n_joints: {n_joints}")
    print(f"  single_obs_dim/history/num_obs: {single_obs_dim}/{config['obs_history_len']}/{num_obs}")
    print(f"  cmd: {format_vec(cmd, 3)}")
    print(f"  height_cmd: {height_cmd:.4f}")
    print(f"  initial base pos: {format_vec(data.qpos[:3], 3)}")
    print(f"  initial base quat: {format_vec(data.qpos[3:7], 4)}")
    print(f"  initial qj: {format_vec(data.qpos[7 : 7 + n_joints], 16)}")
    print(f"  compare_reference: {bool(reference_module)}")
    print(f"  compare_unitree_29_slot: {bool(args.compare_unitree_29_slot)}")
    print(f"  compare_generated_xml: {bool(args.compare_generated_xml)}")
    if generated_model is not None:
        print(
            "  generated xml shape: "
            f"nq={generated_model.nq}, nv={generated_model.nv}, "
            f"nu={generated_model.nu}, nsensor={generated_model.nsensor}, nsensordata={generated_model.nsensordata}"
        )

    policy = None
    if args.no_policy:
        print("  policy: disabled")
    elif policy_path.exists():
        policy = OpenHomiePolicy(policy_path, num_obs=num_obs, num_actions=num_actions)
        print(f"  policy: {policy_path}")
        print(f"  policy input/output shape: {policy.input_shape} -> {policy.output_shape}")
    else:
        print(f"  policy: missing ({policy_path}); running zero-action diagnostic")

    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = config["default_angles"].copy()
    obs_history = collections.deque(maxlen=int(config["obs_history_len"]))
    for _ in range(int(config["obs_history_len"])):
        obs_history.append(np.zeros(single_obs_dim, dtype=np.float32))

    first_policy_printed = False
    control_decimation = int(config["control_decimation"])

    for step in range(max(args.steps, 0)):
        leg_tau = pd_control(
            target_dof_pos,
            data.qpos[7 : 7 + num_actions],
            config["kps"],
            np.zeros_like(config["kps"]),
            data.qvel[6 : 6 + num_actions],
            config["kds"],
        )
        data.ctrl[:num_actions] = leg_tau

        if n_joints > num_actions and model.nu > num_actions:
            arm_count = min(n_joints, model.nu) - num_actions
            arm_tau = pd_control(
                np.zeros(arm_count, dtype=np.float32),
                data.qpos[7 + num_actions : 7 + num_actions + arm_count],
                np.ones(arm_count, dtype=np.float32) * 100.0,
                np.zeros(arm_count, dtype=np.float32),
                data.qvel[6 + num_actions : 6 + num_actions + arm_count],
                np.ones(arm_count, dtype=np.float32) * 0.5,
            )
            data.ctrl[num_actions : num_actions + arm_count] = arm_tau

        mujoco.mj_step(model, data)

        if (step + 1) % control_decimation == 0:
            single_obs = compute_observation(data, config, action, cmd, height_cmd, n_joints)
            if reference_module is not None:
                ref_single_obs, ref_single_obs_dim = reference_module.compute_observation(
                    data, config, action, cmd, height_cmd, n_joints
                )
                if ref_single_obs_dim != single_obs_dim:
                    raise RuntimeError(
                        f"Reference single_obs_dim mismatch: got {ref_single_obs_dim}, expected {single_obs_dim}"
                    )
                max_diff = float(np.max(np.abs(ref_single_obs - single_obs)))
                if max_diff > args.compare_atol:
                    raise RuntimeError(
                        f"OpenHomie reference observation mismatch at step {step + 1}: "
                        f"max_abs_diff={max_diff:.9g}, atol={args.compare_atol:.9g}"
                    )
            if args.compare_unitree_29_slot:
                unitree_single_obs = compute_observation_via_unitree_29_mapping(
                    data, config, action, cmd, height_cmd, n_joints
                )
                unitree_max_diff = float(np.max(np.abs(unitree_single_obs - single_obs)))
                if unitree_max_diff > args.compare_atol:
                    raise RuntimeError(
                        f"Unitree 29-slot observation mismatch at step {step + 1}: "
                        f"max_abs_diff={unitree_max_diff:.9g}, atol={args.compare_atol:.9g}"
                    )
            if generated_model is not None and generated_data is not None:
                copy_openhomie_state_to_generated_29(data, generated_data)
                mujoco.mj_forward(generated_model, generated_data)
                generated_single_obs = compute_observation_from_generated_29_sensors(
                    generated_data, config, action, cmd, height_cmd
                )
                generated_max_diff = float(np.max(np.abs(generated_single_obs - single_obs)))
                if generated_max_diff > args.compare_atol:
                    raise RuntimeError(
                        f"Generated XML sensor-derived observation mismatch at step {step + 1}: "
                        f"max_abs_diff={generated_max_diff:.9g}, atol={args.compare_atol:.9g}"
                    )
            obs_history.append(single_obs)
            obs = np.concatenate(tuple(obs_history)).astype(np.float32)
            if policy is not None:
                action = policy.run(obs)
            else:
                action = np.zeros(num_actions, dtype=np.float32)
            target_dof_pos = action * float(config["action_scale"]) + config["default_angles"]

            if not first_policy_printed:
                print(f"  first obs norm: {float(np.linalg.norm(obs)):.6f}")
                if reference_module is not None:
                    print(f"  reference single_obs max_abs_diff: {max_diff:.9g}")
                if args.compare_unitree_29_slot:
                    print(f"  unitree 29-slot single_obs max_abs_diff: {unitree_max_diff:.9g}")
                if generated_model is not None:
                    print(f"  generated xml sensor single_obs max_abs_diff: {generated_max_diff:.9g}")
                print(f"  first action: {format_vec(action)}")
                print(f"  first target_dof_pos: {format_vec(target_dof_pos)}")
                print(f"  first leg_tau: {format_vec(leg_tau)}")
                first_policy_printed = True

    print(f"  final base pos: {format_vec(data.qpos[:3], 3)}")
    print(f"  final base quat: {format_vec(data.qpos[3:7], 4)}")
    print(f"  final qj: {format_vec(data.qpos[7 : 7 + n_joints], 16)}")
    print(f"  final qvel norm: {float(np.linalg.norm(data.qvel)):.6f}")


if __name__ == "__main__":
    main()

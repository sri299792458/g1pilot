from pathlib import Path

import numpy as np


def quat_rotate_inverse(q, v):
    w, x, y, z = q
    q_conj = np.array([w, -x, -y, -z], dtype=np.float32)
    return np.array(
        [
            v[0] * (q_conj[0] ** 2 + q_conj[1] ** 2 - q_conj[2] ** 2 - q_conj[3] ** 2)
            + v[1] * 2 * (q_conj[1] * q_conj[2] - q_conj[0] * q_conj[3])
            + v[2] * 2 * (q_conj[1] * q_conj[3] + q_conj[0] * q_conj[2]),
            v[0] * 2 * (q_conj[1] * q_conj[2] + q_conj[0] * q_conj[3])
            + v[1] * (q_conj[0] ** 2 - q_conj[1] ** 2 + q_conj[2] ** 2 - q_conj[3] ** 2)
            + v[2] * 2 * (q_conj[2] * q_conj[3] - q_conj[0] * q_conj[1]),
            v[0] * 2 * (q_conj[1] * q_conj[3] - q_conj[0] * q_conj[2])
            + v[1] * 2 * (q_conj[2] * q_conj[3] + q_conj[0] * q_conj[1])
            + v[2] * (q_conj[0] ** 2 - q_conj[1] ** 2 - q_conj[2] ** 2 + q_conj[3] ** 2),
        ],
        dtype=np.float32,
    )


def gravity_orientation(quat_wxyz):
    return quat_rotate_inverse(quat_wxyz, np.array([0.0, 0.0, -1.0], dtype=np.float32))


def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd


def openhomie_single_obs_dim(num_joints, num_actions):
    return 3 + 1 + 3 + 3 + int(num_joints) + int(num_joints) + int(num_actions)


def padded_default_angles(default_angles, num_joints):
    defaults = np.zeros(int(num_joints), dtype=np.float32)
    source = np.asarray(default_angles, dtype=np.float32)
    n = min(len(source), int(num_joints))
    defaults[:n] = source[:n]
    return defaults


def compute_openhomie_observation(
    *,
    command,
    height_cmd,
    omega,
    quat_wxyz,
    qj,
    dqj,
    last_action,
    default_angles,
    cmd_scale,
    ang_vel_scale,
    dof_pos_scale,
    dof_vel_scale,
):
    qj = np.asarray(qj, dtype=np.float32)
    dqj = np.asarray(dqj, dtype=np.float32)
    command = np.asarray(command, dtype=np.float32)
    omega = np.asarray(omega, dtype=np.float32)
    quat_wxyz = np.asarray(quat_wxyz, dtype=np.float32)
    last_action = np.asarray(last_action, dtype=np.float32)
    cmd_scale = np.asarray(cmd_scale, dtype=np.float32)

    if len(qj) != len(dqj):
        raise ValueError(f"qj/dqj length mismatch: {len(qj)} != {len(dqj)}")

    num_joints = len(qj)
    num_actions = len(last_action)
    defaults = padded_default_angles(default_angles, num_joints)
    single_obs = np.zeros(openhomie_single_obs_dim(num_joints, num_actions), dtype=np.float32)

    idx = 0
    single_obs[idx : idx + 3] = command[:3] * cmd_scale
    idx += 3
    single_obs[idx] = float(height_cmd)
    idx += 1
    single_obs[idx : idx + 3] = omega * float(ang_vel_scale)
    idx += 3
    single_obs[idx : idx + 3] = gravity_orientation(quat_wxyz)
    idx += 3
    single_obs[idx : idx + num_joints] = (qj - defaults) * float(dof_pos_scale)
    idx += num_joints
    single_obs[idx : idx + num_joints] = dqj * float(dof_vel_scale)
    idx += num_joints
    single_obs[idx : idx + num_actions] = last_action
    return single_obs


class OpenHomiePolicy:
    def __init__(self, path, *, num_obs, num_actions):
        self.path = Path(path)
        self.num_obs = int(num_obs)
        self.num_actions = int(num_actions)
        self.backend = self.path.suffix.lower()
        self.torch = None
        self.session = None
        self.input_name = None

        if self.backend == ".onnx":
            import onnxruntime as ort

            self.session = ort.InferenceSession(str(self.path), providers=["CPUExecutionProvider"])
            inputs = self.session.get_inputs()
            outputs = self.session.get_outputs()
            if len(inputs) != 1 or len(outputs) != 1:
                raise RuntimeError(f"Expected one ONNX input and output, got {len(inputs)} inputs/{len(outputs)} outputs")
            self.input_name = inputs[0].name
            self.input_shape = inputs[0].shape
            self.output_shape = outputs[0].shape
            self._validate_symbolic_shape(self.input_shape, self.num_obs, "input")
            self._validate_symbolic_shape(self.output_shape, self.num_actions, "output")
            return

        if self.backend == ".pt":
            import torch

            self.torch = torch
            self.model = torch.jit.load(str(self.path), map_location="cpu")
            self.model.eval()
            self.input_shape = ["batch", self.num_obs]
            self.output_shape = ["batch", self.num_actions]
            return

        raise RuntimeError(f"Unsupported policy extension {self.path.suffix!r}; expected .onnx or .pt")

    @staticmethod
    def _validate_symbolic_shape(shape, expected_width, label):
        if len(shape) != 2:
            raise RuntimeError(f"Unexpected ONNX {label} shape {shape}; expected [batch, {expected_width}]")
        width = shape[1]
        if width not in (expected_width, "num_obs", "num_actions", None):
            raise RuntimeError(f"Unexpected ONNX {label} shape {shape}; expected [batch, {expected_width}]")

    def run(self, obs):
        obs = np.asarray(obs, dtype=np.float32).reshape(-1)
        if len(obs) != self.num_obs:
            raise RuntimeError(f"Policy input has {len(obs)} values; expected {self.num_obs}")

        if self.backend == ".onnx":
            action = self.session.run(None, {self.input_name: obs.reshape(1, -1)})[0].reshape(-1).astype(np.float32)
        else:
            with self.torch.no_grad():
                tensor = self.torch.from_numpy(obs).unsqueeze(0)
                action = self.model(tensor).detach().cpu().numpy().reshape(-1).astype(np.float32)

        if len(action) != self.num_actions:
            raise RuntimeError(f"Policy returned {len(action)} actions; expected {self.num_actions}")
        return action

#!/usr/bin/env python3
"""Generate sampled G1 arm reachability maps."""

import argparse
import json
from pathlib import Path
import time

import numpy as np

from ament_index_python.packages import get_package_share_directory
from xbot2_interface import pyxbot2_interface as xbi

from g1pilot.manipulation.opensot_solver import LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, q_init
from g1pilot.utils.joints_names import JOINT_NAMES_ROS

try:
    import pyopensot  # noqa: F401
    from pyopensot_collision.constraints.velocity import CollisionAvoidance
except ImportError:
    CollisionAvoidance = None


SIDE_CONFIG = {
    "left": {
        "joints": LEFT_ARM_JOINTS,
        "frame": "left_hand_point_contact",
    },
    "right": {
        "joints": RIGHT_ARM_JOINTS,
        "frame": "right_hand_point_contact",
    },
}

COLLISION_PAIRS = {
    # Left arm vs torso
    ("left_shoulder_yaw_link", "torso_link"),
    ("left_elbow_link", "torso_link"),
    ("left_wrist_roll_link", "torso_link"),
    ("left_wrist_pitch_link", "torso_link"),
    ("left_wrist_yaw_link", "torso_link"),
    ("left_rubber_hand", "torso_link"),
    # Right arm vs torso
    ("right_shoulder_yaw_link", "torso_link"),
    ("right_elbow_link", "torso_link"),
    ("right_wrist_roll_link", "torso_link"),
    ("right_wrist_pitch_link", "torso_link"),
    ("right_wrist_yaw_link", "torso_link"),
    ("right_rubber_hand", "torso_link"),
    # Hands vs waist/pelvis
    ("left_rubber_hand", "waist_yaw_link"),
    ("right_rubber_hand", "waist_yaw_link"),
    ("left_rubber_hand", "pelvis_contour_link"),
    ("right_rubber_hand", "pelvis_contour_link"),
    # Hands vs legs
    ("left_rubber_hand", "left_hip_pitch_link"),
    ("left_rubber_hand", "left_hip_roll_link"),
    ("left_rubber_hand", "left_hip_yaw_link"),
    ("left_rubber_hand", "left_knee_link"),
    ("left_rubber_hand", "right_hip_pitch_link"),
    ("left_rubber_hand", "right_hip_roll_link"),
    ("left_rubber_hand", "right_hip_yaw_link"),
    ("left_rubber_hand", "right_knee_link"),
    ("right_rubber_hand", "left_hip_pitch_link"),
    ("right_rubber_hand", "left_hip_roll_link"),
    ("right_rubber_hand", "left_hip_yaw_link"),
    ("right_rubber_hand", "left_knee_link"),
    ("right_rubber_hand", "right_hip_pitch_link"),
    ("right_rubber_hand", "right_hip_roll_link"),
    ("right_rubber_hand", "right_hip_yaw_link"),
    ("right_rubber_hand", "right_knee_link"),
}


def packaged_urdf_path(urdf_file):
    path = Path(urdf_file)
    if path.is_absolute() or path.exists():
        return path
    share = Path(get_package_share_directory("g1pilot"))
    return share / "description_files" / "urdf" / urdf_file


def build_q_index_by_motor_id(model):
    q_names = list(model.getQNames())
    return {
        motor_id: q_names.index(joint_name)
        for motor_id, joint_name in JOINT_NAMES_ROS.items()
        if joint_name in q_names
    }


def nominal_q(model, q_index_by_motor_id):
    q = model.getJointPosition().copy()
    q[2] = 0.6756
    q[6] = 1.0
    for motor_id, q_index in q_index_by_motor_id.items():
        if motor_id < len(q_init):
            q[q_index] = float(q_init[motor_id])
    return q


def arm_limits(model, side, q_index_by_motor_id):
    qmin, qmax = model.getJointLimits()
    motor_ids = [joint.value for joint in SIDE_CONFIG[side]["joints"]]
    missing = [mid for mid in motor_ids if mid not in q_index_by_motor_id]
    if missing:
        names = ", ".join(JOINT_NAMES_ROS.get(mid, str(mid)) for mid in missing)
        raise RuntimeError(f"{side} arm joints missing from selected URDF: {names}")
    q_indices = np.array([q_index_by_motor_id[mid] for mid in motor_ids], dtype=int)
    limit_indices = np.array(
        [model.getVIndexFromVName(JOINT_NAMES_ROS[mid]) for mid in motor_ids],
        dtype=int,
    )
    return motor_ids, q_indices, qmin[limit_indices], qmax[limit_indices]


def make_collision_checker(model, urdf_string, enabled, min_distance):
    if not enabled:
        return None
    if CollisionAvoidance is None:
        raise RuntimeError("pyopensot_collision is not available; use --no-collision-filter to skip")
    checker = CollisionAvoidance(
        model,
        max_pairs=len(COLLISION_PAIRS),
        collision_urdf=urdf_string,
    )
    checker.setCollisionList(COLLISION_PAIRS)
    checker.setMaxPairs(len(COLLISION_PAIRS))
    checker.setLinkPairThreshold(min_distance)
    checker.setDetectionThreshold(-1)
    return checker


def min_collision_distance(checker):
    if checker is None:
        return float("inf")
    checker.update()
    witness_points = checker.getOrderedWitnessPointVector()
    if not witness_points:
        return float("inf")
    distances = [
        float(np.linalg.norm(np.asarray(pa) - np.asarray(pb)))
        for pa, pb in witness_points
    ]
    return min(distances) if distances else float("inf")


def sample_side(model, side, q_nominal, q_indices, lo, hi, n_samples, rng, checker,
                min_distance, progress_every):
    frame = SIDE_CONFIG[side]["frame"]
    positions = []
    configs = []
    min_distances = []
    rejected_collision = 0
    started = time.monotonic()

    for i in range(int(n_samples)):
        q = q_nominal.copy()
        arm_q = rng.uniform(lo, hi)
        q[q_indices] = arm_q
        model.setJointPosition(q)
        model.update()

        dmin = min_collision_distance(checker)
        if dmin < min_distance:
            rejected_collision += 1
        else:
            pose = model.getPose(frame, "pelvis")
            positions.append(np.asarray(pose.translation, dtype=np.float64).copy())
            configs.append(arm_q.astype(np.float64).copy())
            min_distances.append(dmin)

        if progress_every and (i + 1) % progress_every == 0:
            elapsed = max(time.monotonic() - started, 1e-9)
            rate = (i + 1) / elapsed
            print(
                f"{side}: {i + 1:,}/{n_samples:,} sampled, "
                f"accepted={len(positions):,}, rejected_collision={rejected_collision:,}, "
                f"{rate:.0f} samples/s"
            )

    return positions, configs, min_distances, rejected_collision


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf-file", default="g1_29dof_lock_waist.urdf")
    parser.add_argument("--samples-per-arm", type=int, default=50000)
    parser.add_argument("--output", default="config/reachability/g1_29dof_lock_waist_reachability.npz")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--min-collision-distance", type=float, default=0.01)
    parser.add_argument("--progress-every", type=int, default=5000)
    parser.add_argument("--no-collision-filter", action="store_true")
    parsed = parser.parse_args(args)

    urdf_path = packaged_urdf_path(parsed.urdf_file)
    urdf_string = urdf_path.read_text()
    model = xbi.ModelInterface2(urdf_string)
    q_index_by_motor_id = build_q_index_by_motor_id(model)
    q_nominal = nominal_q(model, q_index_by_motor_id)
    collision_filter = not parsed.no_collision_filter
    checker = make_collision_checker(
        model,
        urdf_string,
        collision_filter,
        parsed.min_collision_distance,
    )
    rng = np.random.default_rng(parsed.seed)

    all_positions = []
    all_sides = []
    all_configs = []
    all_min_distances = []
    rejected = {}

    for side in ("left", "right"):
        _, q_indices, lo, hi = arm_limits(model, side, q_index_by_motor_id)
        positions, configs, min_distances, rejected_collision = sample_side(
            model=model,
            side=side,
            q_nominal=q_nominal,
            q_indices=q_indices,
            lo=lo,
            hi=hi,
            n_samples=parsed.samples_per_arm,
            rng=rng,
            checker=checker,
            min_distance=parsed.min_collision_distance,
            progress_every=parsed.progress_every,
        )
        all_positions.extend(positions)
        all_sides.extend([side] * len(positions))
        all_configs.extend(configs)
        all_min_distances.extend(min_distances)
        rejected[side] = rejected_collision

    output = Path(parsed.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    metadata = {
        "urdf_file": str(urdf_path),
        "samples_per_arm_requested": parsed.samples_per_arm,
        "seed": parsed.seed,
        "collision_filter": collision_filter,
        "min_collision_distance": parsed.min_collision_distance,
        "rejected_collision": rejected,
        "frames": {
            "left": SIDE_CONFIG["left"]["frame"],
            "right": SIDE_CONFIG["right"]["frame"],
            "reference": "pelvis",
        },
    }

    positions_array = np.asarray(all_positions, dtype=np.float64).reshape(-1, 3)
    configs_array = np.asarray(all_configs, dtype=np.float64).reshape(-1, 7)

    np.savez_compressed(
        output,
        positions=positions_array,
        sides=np.asarray(all_sides),
        configs=configs_array,
        min_collision_distances=np.asarray(all_min_distances, dtype=np.float64),
        metadata_json=json.dumps(metadata, sort_keys=True),
    )
    print(f"Wrote {output} with {len(positions_array):,} accepted samples")
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


UNITREE_MOTOR_SPECS = (
    ("left_hip_pitch", "left_hip_pitch_joint", "-88 88"),
    ("left_hip_roll", "left_hip_roll_joint", "-88 88"),
    ("left_hip_yaw", "left_hip_yaw_joint", "-88 88"),
    ("left_knee", "left_knee_joint", "-139 139"),
    ("left_ankle_pitch", "left_ankle_pitch_joint", "-50 50"),
    ("left_ankle_roll", "left_ankle_roll_joint", "-50 50"),
    ("right_hip_pitch", "right_hip_pitch_joint", "-88 88"),
    ("right_hip_roll", "right_hip_roll_joint", "-88 88"),
    ("right_hip_yaw", "right_hip_yaw_joint", "-88 88"),
    ("right_knee", "right_knee_joint", "-139 139"),
    ("right_ankle_pitch", "right_ankle_pitch_joint", "-50 50"),
    ("right_ankle_roll", "right_ankle_roll_joint", "-50 50"),
    ("waist_yaw", "waist_yaw_joint", "-88 88"),
    ("waist_roll", "waist_roll_joint", "-50 50"),
    ("waist_pitch", "waist_pitch_joint", "-50 50"),
    ("left_shoulder_pitch", "left_shoulder_pitch_joint", "-25 25"),
    ("left_shoulder_roll", "left_shoulder_roll_joint", "-25 25"),
    ("left_shoulder_yaw", "left_shoulder_yaw_joint", "-25 25"),
    ("left_elbow", "left_elbow_joint", "-25 25"),
    ("left_wrist_roll", "left_wrist_roll_joint", "-25 25"),
    ("left_wrist_pitch", "left_wrist_pitch_joint", "-5 5"),
    ("left_wrist_yaw", "left_wrist_yaw_joint", "-5 5"),
    ("right_shoulder_pitch", "right_shoulder_pitch_joint", "-25 25"),
    ("right_shoulder_roll", "right_shoulder_roll_joint", "-25 25"),
    ("right_shoulder_yaw", "right_shoulder_yaw_joint", "-25 25"),
    ("right_elbow", "right_elbow_joint", "-25 25"),
    ("right_wrist_roll", "right_wrist_roll_joint", "-25 25"),
    ("right_wrist_pitch", "right_wrist_pitch_joint", "-5 5"),
    ("right_wrist_yaw", "right_wrist_yaw_joint", "-5 5"),
)

LOCKED_WAIST_JOINTS = {"waist_roll_joint", "waist_pitch_joint"}
ACTIVE_MOTOR_SPECS = tuple(
    spec for spec in UNITREE_MOTOR_SPECS if spec[1] not in LOCKED_WAIST_JOINTS
)


def repo_root():
    return Path(__file__).resolve().parents[1]


def default_openhomie_source():
    return (
        repo_root().parent
        / "reference_repos"
        / "OpenHomie"
        / "HomieRL"
        / "legged_gym"
        / "resources"
        / "robots"
        / "g1_description"
        / "g1.xml"
    )


def default_output():
    return repo_root() / "description_files" / "xml" / "openhomie_g1_29dof.xml"


def make_actuator():
    actuator = ET.Element("actuator")
    for motor_name, joint_name, ctrlrange in ACTIVE_MOTOR_SPECS:
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": motor_name,
                "joint": joint_name,
                "ctrlrange": ctrlrange,
            },
        )
    return actuator


def make_sensor():
    sensor = ET.Element("sensor")
    for motor_name, joint_name, _ in ACTIVE_MOTOR_SPECS:
        ET.SubElement(sensor, "jointpos", {"name": f"{motor_name}_pos", "joint": joint_name})
    for motor_name, joint_name, _ in ACTIVE_MOTOR_SPECS:
        ET.SubElement(sensor, "jointvel", {"name": f"{motor_name}_vel", "joint": joint_name})
    for motor_name, joint_name, _ in ACTIVE_MOTOR_SPECS:
        ET.SubElement(sensor, "jointactuatorfrc", {"name": f"{motor_name}_torque", "joint": joint_name})

    ET.SubElement(sensor, "framequat", {"name": "imu_quat", "objtype": "site", "objname": "imu_in_pelvis"})
    ET.SubElement(sensor, "gyro", {"name": "imu_gyro", "site": "imu_in_pelvis"})
    ET.SubElement(sensor, "accelerometer", {"name": "imu_acc", "site": "imu_in_pelvis"})
    ET.SubElement(sensor, "framepos", {"name": "frame_pos", "objtype": "site", "objname": "imu_in_pelvis"})
    ET.SubElement(sensor, "framelinvel", {"name": "frame_vel", "objtype": "site", "objname": "imu_in_pelvis"})
    ET.SubElement(
        sensor,
        "framequat",
        {"name": "secondary_imu_quat", "objtype": "site", "objname": "imu_in_torso"},
    )
    ET.SubElement(
        sensor,
        "gyro",
        {"name": "secondary_imu_gyro", "site": "imu_in_torso", "noise": "5e-4", "cutoff": "34.9"},
    )
    ET.SubElement(
        sensor,
        "accelerometer",
        {"name": "secondary_imu_acc", "site": "imu_in_torso", "noise": "1e-2", "cutoff": "157"},
    )
    return sensor


def replace_sections(root, section_names, new_sections):
    children = list(root)
    indices = [idx for idx, child in enumerate(children) if child.tag in section_names]
    if not indices:
        raise RuntimeError(f"Could not find any sections to replace: {section_names}")
    insert_at = min(indices)
    for child in children:
        if child.tag in section_names:
            root.remove(child)
    for offset, section in enumerate(new_sections):
        root.insert(insert_at + offset, section)


def generate(source_path, output_path):
    text = source_path.read_text(encoding="utf-8")
    root = ET.fromstring(text)

    root.set("model", "openhomie_g1_locked_waist_unitree_interface")
    compiler = root.find("compiler")
    if compiler is None:
        raise RuntimeError("OpenHomie XML is missing <compiler>")
    compiler.set("meshdir", "../meshes")

    replace_sections(root, {"actuator", "sensor"}, [make_actuator(), make_sensor()])
    ET.indent(root, space="  ")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(ET.tostring(root, encoding="unicode") + "\n", encoding="utf-8")


def validate_with_mujoco(output_path):
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(output_path))
    expected_nu = len(ACTIVE_MOTOR_SPECS)
    if model.nu != expected_nu:
        raise RuntimeError(f"Generated model has {model.nu} actuators, expected {expected_nu}")
    if model.nsensor != 89:
        raise RuntimeError(f"Generated model has {model.nsensor} sensors, expected 89")
    if model.nsensordata != 107:
        raise RuntimeError(f"Generated model has {model.nsensordata} sensor data values, expected 107")

    for joint_name in LOCKED_WAIST_JOINTS:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name) >= 0:
            raise RuntimeError(f"Generated model still has locked waist joint {joint_name!r}")
    return model


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Generate OpenHomie G1 XML with locked waist roll/pitch and a "
            "Unitree-compatible 29-slot DDS interface in the G1Pilot plant."
        )
    )
    parser.add_argument("--source", type=Path, default=default_openhomie_source())
    parser.add_argument("--output", type=Path, default=default_output())
    parser.add_argument("--validate", action="store_true", help="Load generated XML with MuJoCo and verify actuator/sensor counts.")
    args = parser.parse_args()

    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not source.exists():
        raise RuntimeError(f"OpenHomie source XML does not exist: {source}")

    generate(source, output)
    print(f"Generated {output}")

    if args.validate:
        model = validate_with_mujoco(output)
        print(
            "Validated with MuJoCo: "
            f"nq={model.nq}, nv={model.nv}, nu={model.nu}, "
            f"nsensor={model.nsensor}, nsensordata={model.nsensordata}"
        )


if __name__ == "__main__":
    main()

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

HAND_JOINT_SPECS = (
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
)


def repo_root():
    return Path(__file__).resolve().parents[1]


def default_source():
    return (
        repo_root().parent
        / "reference_repos"
        / "GR00T-WholeBodyControl"
        / "gear_sonic"
        / "data"
        / "robots"
        / "g1"
        / "g1_29dof_with_hand_rev_1_0_activatedfinger.xml"
    )


def default_output():
    return repo_root() / "description_files" / "xml" / "openhomie_g1_29dof.xml"


def _joint_actuator_range(root, joint_name):
    joint = root.find(f".//joint[@name='{joint_name}']")
    if joint is None:
        raise RuntimeError(f"Source XML is missing joint {joint_name!r}")
    ctrlrange = joint.attrib.get("actuatorfrcrange")
    if not ctrlrange:
        raise RuntimeError(f"Joint {joint_name!r} is missing actuatorfrcrange")
    return ctrlrange


def make_actuator(root):
    actuator = ET.Element("actuator")
    for motor_name, joint_name, ctrlrange in ACTIVE_MOTOR_SPECS:
        if root.find(f".//joint[@name='{joint_name}']") is None:
            raise RuntimeError(f"Source XML is missing body joint {joint_name!r}")
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": motor_name,
                "joint": joint_name,
                "ctrlrange": ctrlrange,
            },
        )
    for joint_name in HAND_JOINT_SPECS:
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": joint_name,
                "joint": joint_name,
                "ctrlrange": _joint_actuator_range(root, joint_name),
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


def _insert_after(root, after_tag, element):
    children = list(root)
    for idx, child in enumerate(children):
        if child.tag == after_tag:
            root.insert(idx + 1, element)
            return
    root.insert(0, element)


def _remove_top_level(root, tag):
    for child in list(root):
        if child.tag == tag:
            root.remove(child)


def make_statistic():
    return ET.Element("statistic", {"center": "0 0 0.5", "extent": "2.0"})


def make_visual():
    visual = ET.Element("visual")
    ET.SubElement(
        visual,
        "headlight",
        {
            "diffuse": "0.6 0.6 0.6",
            "ambient": "0.3 0.3 0.3",
            "specular": "0 0 0",
        },
    )
    ET.SubElement(visual, "rgba", {"haze": "0.15 0.25 0.35 1"})
    ET.SubElement(visual, "global", {"azimuth": "-130", "elevation": "-20"})
    return visual


def add_scene_assets(root):
    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        _insert_after(root, "default", asset)

    existing_names = {child.attrib.get("name") for child in asset}
    has_skybox = any(child.tag == "texture" and child.attrib.get("type") == "skybox" for child in asset)

    if not has_skybox:
        ET.SubElement(
            asset,
            "texture",
            {
                "type": "skybox",
                "builtin": "gradient",
                "rgb1": "0.3 0.5 0.7",
                "rgb2": "0 0 0",
                "width": "512",
                "height": "3072",
            },
        )
    if "groundplane" not in existing_names:
        ET.SubElement(
            asset,
            "texture",
            {
                "type": "2d",
                "name": "groundplane",
                "builtin": "checker",
                "mark": "edge",
                "rgb1": "0.2 0.3 0.4",
                "rgb2": "0.1 0.2 0.3",
                "markrgb": "0.8 0.8 0.8",
                "width": "300",
                "height": "300",
            },
        )
        ET.SubElement(
            asset,
            "material",
            {
                "name": "groundplane",
                "texture": "groundplane",
                "texuniform": "true",
                "texrepeat": "5 5",
                "reflectance": "0.2",
            },
        )


def add_scene_worldbody(root):
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Source XML is missing <worldbody>")

    for child in list(worldbody):
        if child.attrib.get("name") in {"floor", "sun", "track"}:
            worldbody.remove(child)

    scene_elements = [
        ET.Element(
            "light",
            {
                "name": "sun",
                "pos": "0 0 1.5",
                "dir": "0 0 -1",
                "directional": "true",
            },
        ),
        ET.Element(
            "geom",
            {
                "name": "floor",
                "size": "0 0 0.05",
                "type": "plane",
                "material": "groundplane",
            },
        ),
        ET.Element(
            "camera",
            {
                "name": "track",
                "mode": "trackcom",
                "pos": "2.5 -2.5 1.5",
                "xyaxes": "1 1 0 -0.3 0.3 1",
            },
        ),
    ]
    for element in reversed(scene_elements):
        worldbody.insert(0, element)


def add_scene(root):
    _remove_top_level(root, "statistic")
    _remove_top_level(root, "visual")
    _insert_after(root, "compiler", make_statistic())
    _insert_after(root, "statistic", make_visual())
    add_scene_assets(root)
    add_scene_worldbody(root)


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


def remove_locked_waist_joints(root):
    for joint_name in LOCKED_WAIST_JOINTS:
        joint = root.find(f".//joint[@name='{joint_name}']")
        if joint is None:
            continue
        for parent in root.iter():
            if joint in list(parent):
                parent.remove(joint)
                break
        else:
            raise RuntimeError(f"Could not remove locked waist joint {joint_name!r}")


def generate(source_path, output_path):
    text = source_path.read_text(encoding="utf-8")
    root = ET.fromstring(text)

    root.set("model", "g1pilot_g1_locked_waist_dex3_unitree_interface")
    compiler = root.find("compiler")
    if compiler is None:
        raise RuntimeError("Source XML is missing <compiler>")
    compiler.set("meshdir", "../meshes")

    remove_locked_waist_joints(root)
    add_scene(root)
    replace_sections(root, {"actuator", "sensor"}, [make_actuator(root), make_sensor()])
    ET.indent(root, space="  ")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(ET.tostring(root, encoding="unicode") + "\n", encoding="utf-8")


def validate_with_mujoco(output_path):
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(output_path))
    expected_nu = len(ACTIVE_MOTOR_SPECS) + len(HAND_JOINT_SPECS)
    if model.nu != expected_nu:
        raise RuntimeError(f"Generated model has {model.nu} actuators, expected {expected_nu}")
    if model.nsensor != 89:
        raise RuntimeError(f"Generated model has {model.nsensor} sensors, expected 89")
    if model.nsensordata != 107:
        raise RuntimeError(f"Generated model has {model.nsensordata} sensor data values, expected 107")
    for obj_type, name in (
        (mujoco.mjtObj.mjOBJ_GEOM, "floor"),
        (mujoco.mjtObj.mjOBJ_LIGHT, "sun"),
        (mujoco.mjtObj.mjOBJ_CAMERA, "track"),
    ):
        if mujoco.mj_name2id(model, obj_type, name) < 0:
            raise RuntimeError(f"Generated model is missing scene object {name!r}")

    for joint_name in LOCKED_WAIST_JOINTS:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name) >= 0:
            raise RuntimeError(f"Generated model still has locked waist joint {joint_name!r}")
    for joint_name in HAND_JOINT_SPECS:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name) < 0:
            raise RuntimeError(f"Generated model is missing Dex3 joint {joint_name!r}")
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name) < 0:
            raise RuntimeError(f"Generated model is missing Dex3 actuator {joint_name!r}")
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        dof_id = int(model.jnt_dofadr[joint_id])
        damping = float(model.dof_damping[dof_id])
        if abs(damping - 0.05) > 1e-9:
            raise RuntimeError(
                f"Dex3 joint {joint_name!r} has damping {damping}; expected "
                "GR00T activated-finger damping 0.05"
            )
    return model


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Generate OpenHomie G1 XML with locked waist roll/pitch and a "
            "Unitree-compatible body DDS interface plus Dex3 hand actuators "
            "for the G1Pilot plant. The source is the GR00T activated-finger "
            "Rev 1.0 model so finger damping, limits, geometry, and inertials "
            "come from the simulation-stable hand model."
        )
    )
    parser.add_argument("--source", type=Path, default=default_source())
    parser.add_argument("--output", type=Path, default=default_output())
    parser.add_argument("--validate", action="store_true", help="Load generated XML with MuJoCo and verify actuator/sensor counts.")
    args = parser.parse_args()

    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not source.exists():
        raise RuntimeError(f"Source XML does not exist: {source}")

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

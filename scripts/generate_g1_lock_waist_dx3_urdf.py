#!/usr/bin/env python3
"""Generate the locked-waist Dex3 visualization URDF.

The source of truth is G1Pilot's local Dex3 URDF. This script only applies the
lab hardware constraint that waist roll and waist pitch are physically locked.
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = REPO_ROOT / "description_files/urdf/g1_29dof_dx3.urdf"
DEFAULT_OUTPUT = REPO_ROOT / "description_files/urdf/g1_29dof_lock_waist_dx3.urdf"

LOCKED_JOINTS = ("waist_roll_joint", "waist_pitch_joint")
REMOVE_FROM_FIXED_JOINT = {
    "axis",
    "limit",
    "dynamics",
    "mimic",
    "calibration",
    "safety_controller",
}
REQUIRED_LINKS = (
    "livox_frame",
    "left_hand_point_contact",
    "right_hand_point_contact",
)
REQUIRED_DEX3_JOINTS = (
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


def _named_elements(root: ET.Element, tag: str) -> dict[str, ET.Element]:
    elements: dict[str, ET.Element] = {}
    for element in root.findall(tag):
        name = element.get("name")
        if name:
            if name in elements:
                raise ValueError(f"duplicate <{tag}> name {name!r}")
            elements[name] = element
    return elements


def _lock_joint(joint: ET.Element) -> None:
    joint.set("type", "fixed")
    for child in list(joint):
        if child.tag in REMOVE_FROM_FIXED_JOINT:
            joint.remove(child)


def generate(input_path: Path, output_path: Path) -> None:
    tree = ET.parse(input_path)
    root = tree.getroot()
    if root.tag != "robot":
        raise ValueError(f"{input_path} root is <{root.tag}>, expected <robot>")

    root.set("name", "g1_29dof_lock_waist_dx3")
    joints = _named_elements(root, "joint")
    links = _named_elements(root, "link")

    missing_locked_joints = [name for name in LOCKED_JOINTS if name not in joints]
    if missing_locked_joints:
        raise ValueError(f"source URDF is missing joints: {missing_locked_joints}")

    for name in LOCKED_JOINTS:
        _lock_joint(joints[name])

    missing_links = [name for name in REQUIRED_LINKS if name not in links]
    if missing_links:
        raise ValueError(f"source URDF is missing required links: {missing_links}")

    missing_dex3 = [name for name in REQUIRED_DEX3_JOINTS if name not in joints]
    if missing_dex3:
        raise ValueError(f"source URDF is missing Dex3 joints: {missing_dex3}")

    for name in LOCKED_JOINTS:
        joint = joints[name]
        if joint.get("type") != "fixed":
            raise ValueError(f"{name} was not converted to fixed")
        removed_children = [child.tag for child in joint if child.tag in REMOVE_FROM_FIXED_JOINT]
        if removed_children:
            raise ValueError(f"{name} still has invalid fixed-joint children: {removed_children}")

    ET.indent(tree, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True, short_empty_elements=True)
    output_path.write_text(output_path.read_text(encoding="utf-8") + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    generate(args.input, args.output)
    print(f"generated {args.output} from {args.input}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

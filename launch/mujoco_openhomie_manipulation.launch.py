from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

import os

visual_urdf_by_hand_model = {
    "dummy": "g1_29dof_lock_waist.urdf",
    "dex3": "g1_29dof_lock_waist_dx3.urdf",
}


def _pkg_launch(filename):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("g1pilot"), "launch", filename)
    )


def _launch_setup(context):
    interface = LaunchConfiguration("interface")
    domain_id = LaunchConfiguration("domain_id")
    hand_model = LaunchConfiguration("hand_model").perform(context).strip().lower()
    if hand_model not in visual_urdf_by_hand_model:
        raise RuntimeError(
            f"Unknown hand_model {hand_model!r}. Allowed values: "
            f"{', '.join(visual_urdf_by_hand_model)}"
        )

    visual_urdf_file = visual_urdf_by_hand_model[hand_model]
    enable_dx3 = "true" if hand_model == "dex3" else "false"

    return [
        IncludeLaunchDescription(
            _pkg_launch("robot_state_launcher.launch.py"),
            launch_arguments={
                "interface": interface,
                "domain_id": domain_id,
                "use_robot": "true",
                "publish_joint_states": "true",
                "mola_fixed_publish_tf": "false",
                "hand_model": hand_model,
                "urdf_file": visual_urdf_file,
            }.items(),
        ),

        IncludeLaunchDescription(
            _pkg_launch("manipulation_launcher.launch.py"),
            launch_arguments={
                "interface": interface,
                "domain_id": domain_id,
                "use_robot": "false",
                "publish_arm_sdk": "true",
                "send_cmds_to_robot": "true",
                "publish_joint_states_opensot": "false",
                "start_robot_state_publisher": "false",
                "enable_dx3": enable_dx3,
                "urdf_file": visual_urdf_file,
                "opensot_urdf_file": "g1_29dof_lock_waist.urdf",
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value=EnvironmentVariable("G1_SIM_INTERFACE", default_value="lo")),
        DeclareLaunchArgument("domain_id", default_value=EnvironmentVariable("G1_UNITREE_DOMAIN_ID", default_value="1")),
        DeclareLaunchArgument(
            "hand_model",
            default_value="dex3",
            description="Visual/control hand mode for the simulator: dex3 or dummy",
        ),
        OpaqueFunction(function=_launch_setup),
    ])

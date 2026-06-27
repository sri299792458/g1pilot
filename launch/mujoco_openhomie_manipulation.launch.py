from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

import os


def _pkg_launch(filename):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("g1pilot"), "launch", filename)
    )


def generate_launch_description():
    interface = LaunchConfiguration("interface")
    domain_id = LaunchConfiguration("domain_id")
    policy_path = LaunchConfiguration("policy_path")
    allow_zero_policy = LaunchConfiguration("allow_zero_policy")

    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value=EnvironmentVariable("G1_SIM_INTERFACE", default_value="lo")),
        DeclareLaunchArgument("domain_id", default_value=EnvironmentVariable("G1_UNITREE_DOMAIN_ID", default_value="1")),
        DeclareLaunchArgument("policy_path", default_value=EnvironmentVariable("OPENHOMIE_POLICY_PATH", default_value="")),
        DeclareLaunchArgument("allow_zero_policy", default_value="false"),

        IncludeLaunchDescription(
            _pkg_launch("mujoco_openhomie_stand.launch.py"),
            launch_arguments={
                "interface": interface,
                "domain_id": domain_id,
                "policy_path": policy_path,
                "allow_zero_policy": allow_zero_policy,
                "mode": "stand",
            }.items(),
        ),

        IncludeLaunchDescription(
            _pkg_launch("robot_state_launcher.launch.py"),
            launch_arguments={
                "interface": interface,
                "domain_id": domain_id,
                "use_robot": "true",
                "publish_joint_states": "true",
                "mola_fixed_publish_tf": "false",
                "urdf_file": "g1_29dof_lock_waist.urdf",
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
                "urdf_file": "g1_29dof_lock_waist.urdf",
            }.items(),
        ),
    ])

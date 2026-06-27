from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os


def generate_launch_description():
    default_config_file = os.path.join(
        get_package_share_directory("g1pilot"), "config", "openhomie_g1_policy.yaml"
    )
    interface = LaunchConfiguration("interface")
    domain_id = LaunchConfiguration("domain_id")
    policy_path = LaunchConfiguration("policy_path")
    allow_zero_policy = LaunchConfiguration("allow_zero_policy")
    mode = LaunchConfiguration("mode")

    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value=EnvironmentVariable("G1_SIM_INTERFACE", default_value="lo")),
        DeclareLaunchArgument("domain_id", default_value=EnvironmentVariable("G1_UNITREE_DOMAIN_ID", default_value="1")),
        DeclareLaunchArgument("policy_path", default_value=EnvironmentVariable("OPENHOMIE_POLICY_PATH", default_value="")),
        DeclareLaunchArgument("allow_zero_policy", default_value="false"),
        DeclareLaunchArgument("mode", default_value="stand"),

        Node(
            package="g1pilot",
            executable="openhomie_lowcmd_base",
            name="openhomie_lowcmd_base",
            output="screen",
            parameters=[
                default_config_file,
                {
                    "interface": ParameterValue(interface, value_type=str),
                    "domain_id": ParameterValue(domain_id, value_type=int),
                    "policy_path": ParameterValue(policy_path, value_type=str),
                    "allow_zero_policy": ParameterValue(allow_zero_policy, value_type=bool),
                    "mode": ParameterValue(mode, value_type=str),
                },
            ],
        ),

        Node(
            package="g1pilot",
            executable="arm_sdk_lowcmd_overlay",
            name="arm_sdk_lowcmd_overlay",
            output="screen",
            parameters=[
                default_config_file,
                {
                    "interface": ParameterValue(interface, value_type=str),
                    "domain_id": ParameterValue(domain_id, value_type=int),
                },
            ],
        ),
    ])

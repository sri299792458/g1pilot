from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

package_name = "g1pilot"
default_urdf_file_name = "g1_29dof_lock_waist.urdf"
default_opensot_urdf_file_name = "g1_29dof_lock_waist.urdf"
default_reachability_map_file_name = "g1_29dof_lock_waist_reachability.npz"
allowed_urdf_file_names = (
    "g1_29dof_lock_waist.urdf",
    "g1_29dof_lock_waist_dx3.urdf",
    "g1_29dof.urdf",
    "g1_29dof_dx3.urdf",
    "g1_29dof_upperbody.urdf",
    "g1_29dof_dx3_upperbody.urdf",
)

def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")

def _validate_robot_interface(context):
    use_robot = _as_bool(LaunchConfiguration("use_robot").perform(context))
    publish_arm_sdk = _as_bool(LaunchConfiguration("publish_arm_sdk").perform(context))
    interface = LaunchConfiguration("interface").perform(context).strip()
    if (use_robot or publish_arm_sdk) and not interface:
        raise RuntimeError(
            "G1_INTERFACE/interface is required when use_robot:=true or publish_arm_sdk:=true. "
            "Set it with `export G1_INTERFACE=<iface>`, pass `interface:=<iface>`, "
            "or launch with `use_robot:=false publish_arm_sdk:=false` for offline mode."
        )
    return []

def _load_robot_description(context, launch_arg_name):
    urdf_file_name = LaunchConfiguration(launch_arg_name).perform(context).strip()
    if os.path.basename(urdf_file_name) != urdf_file_name:
        raise RuntimeError(
            f"{launch_arg_name} must be a packaged URDF file name, not a path. "
            f"Allowed values: {', '.join(allowed_urdf_file_names)}"
        )
    if urdf_file_name not in allowed_urdf_file_names:
        raise RuntimeError(
            f"Unknown {launch_arg_name} {urdf_file_name!r}. "
            f"Allowed values: {', '.join(allowed_urdf_file_names)}"
        )

    urdf = os.path.join(
        get_package_share_directory(package_name), "description_files/urdf", urdf_file_name
    )
    with open(urdf, "r") as infp:
        robot_desc = infp.read()
    return urdf, robot_desc

def _launch_setup(context):
    interface = LaunchConfiguration("interface")
    domain_id = LaunchConfiguration("domain_id")
    use_robot = LaunchConfiguration("use_robot")
    arm_controlled = LaunchConfiguration("arm_controlled")
    enable_collision_avoidance = LaunchConfiguration("enable_collision_avoidance")
    send_cmds_to_robot = LaunchConfiguration("send_cmds_to_robot")
    publish_arm_sdk = LaunchConfiguration("publish_arm_sdk")
    publish_joint_states_opensot = LaunchConfiguration("publish_joint_states_opensot")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")
    enable_reachability_gate = LaunchConfiguration("enable_reachability_gate")
    enable_dx3 = LaunchConfiguration("enable_dx3")
    reachability_map_file = LaunchConfiguration("reachability_map_file")
    reachability_query_radius = LaunchConfiguration("reachability_query_radius")
    reachability_min_neighbors = LaunchConfiguration("reachability_min_neighbors")
    reachability_snap_rejected_marker = LaunchConfiguration("reachability_snap_rejected_marker")

    urdf, robot_desc = _load_robot_description(context, "urdf_file")
    _, opensot_robot_desc = _load_robot_description(context, "opensot_urdf_file")
    use_robot_bool = _as_bool(LaunchConfiguration("use_robot").perform(context))
    publish_arm_sdk_bool = _as_bool(LaunchConfiguration("publish_arm_sdk").perform(context))
    enable_hand_dds = use_robot_bool or publish_arm_sdk_bool

    return [
        Node(
            condition=IfCondition(start_robot_state_publisher),
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_desc,
            }],
            arguments=[urdf],
        ),

        Node(
            package='g1pilot',
            executable='opensot_solver',
            name='opensot_solver',
            parameters=[{
                'interface': interface,
                'domain_id': ParameterValue(domain_id, value_type=int),
                'arm_controlled': ParameterValue(arm_controlled, value_type=str),
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'enable_collision_avoidance': ParameterValue(enable_collision_avoidance, value_type=bool),
                'send_cmds_to_robot': ParameterValue(send_cmds_to_robot, value_type=bool),
                'publish_arm_sdk': ParameterValue(publish_arm_sdk, value_type=bool),
                'publish_joint_states_opensot': ParameterValue(publish_joint_states_opensot, value_type=bool),
                'robot_description': opensot_robot_desc,
                'enable_reachability_gate': ParameterValue(enable_reachability_gate, value_type=bool),
                'reachability_map_file': ParameterValue(reachability_map_file, value_type=str),
                'reachability_query_radius': ParameterValue(reachability_query_radius, value_type=float),
                'reachability_min_neighbors': ParameterValue(reachability_min_neighbors, value_type=int),
                'reachability_snap_rejected_marker': ParameterValue(
                    reachability_snap_rejected_marker, value_type=bool
                ),
            }],
            output='screen'
        ),

        Node(
            condition=IfCondition(enable_dx3),
            package='g1pilot',
            executable='dx3_controller',
            name='dx3_controller',
            parameters=[{
                'arm_controlled': ParameterValue(arm_controlled, value_type=str),
                'interface': ParameterValue(interface, value_type=str),
                'domain_id': ParameterValue(domain_id, value_type=int),
                'enable_dds': enable_hand_dds,
                'send_commands': ParameterValue(send_cmds_to_robot, value_type=bool),
            }],
            output='screen'
        ),
    ]

def generate_launch_description():
    default_reachability_map_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "reachability",
        default_reachability_map_file_name,
    )

    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value=EnvironmentVariable("G1_INTERFACE", default_value="")),
        DeclareLaunchArgument("domain_id", default_value=EnvironmentVariable("G1_UNITREE_DOMAIN_ID", default_value="0")),
        DeclareLaunchArgument("use_robot", default_value="true"),
        DeclareLaunchArgument(
            "arm_controlled",
            default_value="both",
            description=(
                "Which arm side to control: left, right, or both. "
                "This selects OpenSoT hand tasks and DX3 hand interfaces."
            ),
        ),
        DeclareLaunchArgument("enable_collision_avoidance", default_value="true"),
        DeclareLaunchArgument("send_cmds_to_robot", default_value="true"),
        DeclareLaunchArgument(
            "publish_arm_sdk",
            default_value="false",
            description=(
                "Publish OpenSoT arm commands on rt/arm_sdk even when use_robot=false. "
                "Use only for simulator DDS on loopback."
            ),
        ),
        DeclareLaunchArgument("publish_joint_states_opensot", default_value="false"),
        DeclareLaunchArgument("enable_reachability_gate", default_value="true"),
        DeclareLaunchArgument(
            "enable_dx3",
            default_value="true",
            description="Start the Dex3 hand DDS/ROS controller",
        ),
        DeclareLaunchArgument("reachability_map_file", default_value=default_reachability_map_file),
        DeclareLaunchArgument("reachability_query_radius", default_value="0.04"),
        DeclareLaunchArgument("reachability_min_neighbors", default_value="1"),
        DeclareLaunchArgument("reachability_snap_rejected_marker", default_value="true"),
        DeclareLaunchArgument(
            "start_robot_state_publisher",
            default_value="true",
            description="Start robot_state_publisher so opensot_solver can fetch robot_description",
        ),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=default_urdf_file_name,
            description="Packaged G1 URDF file to use when starting robot_state_publisher",
        ),
        DeclareLaunchArgument(
            "opensot_urdf_file",
            default_value=default_opensot_urdf_file_name,
            description="Packaged G1 URDF file used by OpenSoT. Keep Dex3 fingers out unless explicitly tested.",
        ),
        OpaqueFunction(function=_validate_robot_interface),
        OpaqueFunction(function=_launch_setup),
    ])

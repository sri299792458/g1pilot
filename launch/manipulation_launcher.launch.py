from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

package_name = "g1pilot"
default_urdf_file_name = "g1_29dof.urdf"
allowed_urdf_file_names = (
    "g1_29dof.urdf",
    "g1_29dof_dx3.urdf",
    "g1_29dof_upperbody.urdf",
    "g1_29dof_dx3_upperbody.urdf",
)

def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")

def _validate_robot_interface(context):
    use_robot = _as_bool(LaunchConfiguration("use_robot").perform(context))
    interface = LaunchConfiguration("interface").perform(context).strip()
    if use_robot and not interface:
        raise RuntimeError(
            "G1_INTERFACE is required when use_robot:=true. "
            "Set it with `export G1_INTERFACE=<iface>`, pass `interface:=<iface>`, "
            "or launch with `use_robot:=false` for offline mode."
        )
    return []

def _load_robot_description(context):
    urdf_file_name = LaunchConfiguration("urdf_file").perform(context).strip()
    if os.path.basename(urdf_file_name) != urdf_file_name:
        raise RuntimeError(
            "urdf_file must be a packaged URDF file name, not a path. "
            f"Allowed values: {', '.join(allowed_urdf_file_names)}"
        )
    if urdf_file_name not in allowed_urdf_file_names:
        raise RuntimeError(
            f"Unknown urdf_file {urdf_file_name!r}. "
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
    use_robot = LaunchConfiguration("use_robot")
    arm_controlled = LaunchConfiguration("arm_controlled")
    enable_collision_avoidance = LaunchConfiguration("enable_collision_avoidance")
    send_cmds_to_robot = LaunchConfiguration("send_cmds_to_robot")
    publish_joint_states_opensot = LaunchConfiguration("publish_joint_states_opensot")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")

    urdf, robot_desc = _load_robot_description(context)

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
                'arm_controlled': ParameterValue(arm_controlled, value_type=str),
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'enable_collision_avoidance': ParameterValue(enable_collision_avoidance, value_type=bool),
                'send_cmds_to_robot': ParameterValue(send_cmds_to_robot, value_type=bool),
                'publish_joint_states_opensot': ParameterValue(publish_joint_states_opensot, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='dx3_controller',
            name='dx3_controller',
            parameters=[{
                'arm_controlled': ParameterValue(arm_controlled, value_type=str),
                'interface': ParameterValue(interface, value_type=str),
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'send_commands': ParameterValue(send_cmds_to_robot, value_type=bool),
            }],
            output='screen'
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value=EnvironmentVariable("G1_INTERFACE", default_value="")),
        DeclareLaunchArgument("use_robot", default_value="true"),
        DeclareLaunchArgument(
            "arm_controlled",
            default_value="both",
            description=(
                "Which arm side to control: left, right, or both. "
                "This selects OpenSoT hand tasks and DX3 hand interfaces."
            ),
        ),
        DeclareLaunchArgument("enable_collision_avoidance", default_value="false"),
        DeclareLaunchArgument("send_cmds_to_robot", default_value="true"),
        DeclareLaunchArgument("publish_joint_states_opensot", default_value="false"),
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
        OpaqueFunction(function=_validate_robot_interface),
        OpaqueFunction(function=_launch_setup),
    ])

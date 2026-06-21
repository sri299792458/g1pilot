#!/home/.base/bin/python3

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue
import os

package_name = "g1pilot"
default_urdf_file_name = "g1_29dof.urdf"
allowed_urdf_file_names = (
    "g1_29dof.urdf",
    "g1_29dof_dx3.urdf",
    "g1_29dof_upperbody.urdf",
    "g1_29dof_dx3_upperbody.urdf",
)
rviz_config_file_name = "29dof.rviz"

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

    package_share = get_package_share_directory(package_name)
    urdf = os.path.join(
        package_share, "description_files/urdf", urdf_file_name
    )
    with open(urdf, "r") as infp:
        robot_desc = infp.read()
    return package_share, urdf, robot_desc

def _launch_setup(context):
    package_share, urdf, robot_desc = _load_robot_description(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_robot = LaunchConfiguration("use_robot")
    publish_joint_states = LaunchConfiguration("publish_joint_states")
    interface = LaunchConfiguration("interface")
    sim_rate_hz = LaunchConfiguration("sim_rate_hz")
    mola_fixed_publish_tf = LaunchConfiguration("mola_fixed_publish_tf")

    rviz_config = os.path.join(package_share, "config", rviz_config_file_name)

    return [
        Node(
            package='g1pilot',
            executable='robot_state',
            name='robot_state',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'sim_rate_hz': ParameterValue(sim_rate_hz, value_type=float),
                'publish_joint_states': ParameterValue(publish_joint_states, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='mola_fixed',
            name='mola_fixed',
            parameters=[{
                'publish_tf': ParameterValue(mola_fixed_publish_tf, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "robot_description": robot_desc
            }],
            arguments=[urdf],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                rviz_config
            ],
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use simulation (Gazebo) clock if true"),
        DeclareLaunchArgument("use_robot", default_value="true",
                              description="Connect to real robot if true"),
        DeclareLaunchArgument("publish_joint_states", default_value="false",
                              description="Publish joint_states from node"),
        DeclareLaunchArgument("interface", default_value=EnvironmentVariable("G1_INTERFACE", default_value=""),
                              description="Network interface for Unitree SDK"),
        DeclareLaunchArgument("sim_rate_hz", default_value="50.0",
                              description="Simulation rate when use_robot=false"),
        DeclareLaunchArgument("mola_fixed_publish_tf", default_value="true",
                              description="Whether mola_fixed publishes map -> pelvis TF"),
        DeclareLaunchArgument("arm_controlled", default_value="both",
                                description="Which arm to control: 'left', 'right', or 'both'"),
        DeclareLaunchArgument("urdf_file", default_value=default_urdf_file_name,
                              description="Packaged G1 URDF file to publish"),
        OpaqueFunction(function=_validate_robot_interface),
        OpaqueFunction(function=_launch_setup),
    ])

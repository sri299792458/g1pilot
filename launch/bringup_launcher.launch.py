from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os

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

def generate_launch_description():
    pkg1_share = FindPackageShare('g1pilot').find('g1pilot')
    default_reachability_map_file = os.path.join(
        pkg1_share,
        "config",
        "reachability",
        "g1_29dof_lock_waist_reachability.npz",
    )

    interface = LaunchConfiguration("interface")
    use_robot = LaunchConfiguration("use_robot")
    arm_controlled = LaunchConfiguration("arm_controlled")
    urdf_file = LaunchConfiguration("urdf_file")
    enable_reachability_gate = LaunchConfiguration("enable_reachability_gate")
    reachability_map_file = LaunchConfiguration("reachability_map_file")
    reachability_query_radius = LaunchConfiguration("reachability_query_radius")
    reachability_min_neighbors = LaunchConfiguration("reachability_min_neighbors")
    reachability_snap_rejected_marker = LaunchConfiguration("reachability_snap_rejected_marker")

    navigation_launcher = os.path.join(pkg1_share, 'launch', 'navigation_launcher.launch.py')
    robot_state_launcher = os.path.join(pkg1_share, 'launch', 'robot_state_launcher.launch.py')
    teleoperation_launcher = os.path.join(pkg1_share, 'launch', 'teleoperation_launcher.launch.py')
    manipulation_launcher = os.path.join(pkg1_share, 'launch', 'manipulation_launcher.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument("enable_collision_avoidance", default_value="true"),
        DeclareLaunchArgument("enable_reachability_gate", default_value="true"),
        DeclareLaunchArgument("reachability_map_file", default_value=default_reachability_map_file),
        DeclareLaunchArgument("reachability_query_radius", default_value="0.04"),
        DeclareLaunchArgument("reachability_min_neighbors", default_value="1"),
        DeclareLaunchArgument("reachability_snap_rejected_marker", default_value="true"),
        DeclareLaunchArgument("use_robot", default_value="true"),
        DeclareLaunchArgument(
            "arm_controlled",
            default_value="both",
            description="Which arm side to control: left, right, or both",
        ),
        DeclareLaunchArgument(
            "interface",
            default_value=EnvironmentVariable("G1_INTERFACE", default_value=""),
            description="Network interface for Unitree SDK",
        ),
        DeclareLaunchArgument(
            "urdf_file",
            default_value="g1_29dof_lock_waist.urdf",
            description="Packaged G1 URDF file to publish",
        ),
        OpaqueFunction(function=_validate_robot_interface),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launcher),
            launch_arguments={
                "interface": interface,
                "use_robot": use_robot,
                "arm_controlled": arm_controlled,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_launcher),
            launch_arguments={
                'interface': interface,
                'use_robot': use_robot,
                'publish_joint_states': 'true',
                'urdf_file': urdf_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleoperation_launcher),
            launch_arguments=[("interface", interface)],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manipulation_launcher),
            launch_arguments={
                'interface': interface,
                'use_robot': use_robot,
                'arm_controlled': arm_controlled,
                'enable_collision_avoidance': LaunchConfiguration('enable_collision_avoidance'),
                'enable_reachability_gate': enable_reachability_gate,
                'reachability_map_file': reachability_map_file,
                'reachability_query_radius': reachability_query_radius,
                'reachability_min_neighbors': reachability_min_neighbors,
                'reachability_snap_rejected_marker': reachability_snap_rejected_marker,
                'send_cmds_to_robot': 'true',
                'publish_joint_states_opensot': 'false',
                'start_robot_state_publisher': 'false',
                'urdf_file': urdf_file,
            }.items(),
        ),
    ])

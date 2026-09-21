#!/usr/bin/env python3
from copy import deepcopy
import subprocess
import threading
import time
import numpy as np
import array

from tf2_ros import TransformBroadcaster
from xbot2_interface import pyxbot2_interface as xbi
from xbot2_interface import pyxbot2_collision
from xbot2_interface import pyaffine3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

try:
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:
    RCLError = None

from geometry_msgs.msg import PoseStamped, TransformStamped, WrenchStamped,Point
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState

from visualization_msgs.msg import (
    InteractiveMarkerControl,
    InteractiveMarker,
    InteractiveMarkerFeedback,
    Marker,
)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler

from scipy.spatial.transform import Rotation as R

import pyopensot as pysot
from pyopensot.tasks.velocity import Postural, Cartesian, CoM
from pyopensot.constraints.velocity import JointLimits, VelocityLimits

try:
    from pyopensot_collision.constraints.velocity import CollisionAvoidance
except ImportError:
    CollisionAvoidance = None

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from g1pilot.utils.common import (
    MotorState,
    G1_29_JointArmIndex,
    G1_29_JointWristIndex,
    G1_29_JointWeakIndex,
    G1_29_JointWaistIndex,
    G1_29_JointIndex,
    DataBuffer,
)
from g1pilot.utils.joints_names import JOINT_NAMES_ROS
from g1pilot.manipulation.reachability_map import ReachabilityMap

G1_NUM_MOTOR = 29 # 12 body + 17 arm

LEFT_ARM_JOINTS = (
    G1_29_JointArmIndex.kLeftShoulderPitch,
    G1_29_JointArmIndex.kLeftShoulderRoll,
    G1_29_JointArmIndex.kLeftShoulderYaw,
    G1_29_JointArmIndex.kLeftElbow,
    G1_29_JointArmIndex.kLeftWristRoll,
    G1_29_JointArmIndex.kLeftWristPitch,
    G1_29_JointArmIndex.kLeftWristyaw,
)

RIGHT_ARM_JOINTS = (
    G1_29_JointArmIndex.kRightShoulderPitch,
    G1_29_JointArmIndex.kRightShoulderRoll,
    G1_29_JointArmIndex.kRightShoulderYaw,
    G1_29_JointArmIndex.kRightElbow,
    G1_29_JointArmIndex.kRightWristRoll,
    G1_29_JointArmIndex.kRightWristPitch,
    G1_29_JointArmIndex.kRightWristYaw,
)

RIGHT_HAND_GOAL_TOPIC = "/g1pilot/hand_goal/right"
LEFT_HAND_GOAL_TOPIC = "/g1pilot/hand_goal/left"
ARMS_ENABLED_TOPIC = "/g1pilot/arms/enabled"
ARMS_HOME_TOPIC = "/g1pilot/arms/home"

q_init = [
        -0.1,
    0.0,
    0.0,  # hips
    0.432,  # knee
    -0.317,
    0.0,  # ankles
    -0.1,
    0.0,
    0.0,  # hips
    0.432,  # knee
    -0.317,
    0.0,  # ankles
    0.0,
    0.0,
    0.0,  # waist
    0.3,
    0.25,
    0.0,
    1.0,
    0.15,
    0.0,
    0.0,  # arm
    0.3,
    -0.25,
    0.0,
    1.0,
    0.15,
    0.0,
    0.0,
]  # arm


class Mode:
    PR = 0
    AB = 1


class G1CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__("g1_collision_avoidance_node")
        self.get_logger().info("Starting G1 Collision Avoidance Node")

        self.declare_parameter("use_robot", True)
        self.declare_parameter("enable_collision_avoidance", False)
        self.declare_parameter("interface", "")
        self.declare_parameter("domain_id", 0)
        self.declare_parameter("send_cmds_to_robot", True)
        self.declare_parameter("publish_arm_sdk", False)
        self.declare_parameter("publish_joint_states_opensot", False)
        self.declare_parameter("robot_description", "")
        self.declare_parameter("robot_description_timeout_s", 10.0)
        self.declare_parameter("arm_controlled", "both")
        self.declare_parameter("enable_reachability_gate", False)
        self.declare_parameter("reachability_map_file", "")
        self.declare_parameter("reachability_query_radius", 0.04)
        self.declare_parameter("reachability_min_neighbors", 1)
        self.declare_parameter("reachability_snap_rejected_marker", False)
        self.interface = str(self.get_parameter("interface").value)
        self.domain_id = int(self.get_parameter("domain_id").value)
        self.use_robot = bool(self.get_parameter("use_robot").value)
        self.declare_parameter("publish_pelvis_tf", not self.use_robot)
        self.enable_collision_avoidance = bool(self.get_parameter("enable_collision_avoidance").value)
        self.send_cmds_to_robot = bool(self.get_parameter("send_cmds_to_robot").value)
        self.publish_arm_sdk = bool(self.get_parameter("publish_arm_sdk").value) or self.use_robot
        self.publish_joint_states_opensot = bool(self.get_parameter("publish_joint_states_opensot").value)
        robot_description_param = self.get_parameter("robot_description").get_parameter_value().string_value
        self.publish_pelvis_tf = bool(self.get_parameter("publish_pelvis_tf").value)
        self.robot_description_timeout_s = float(self.get_parameter("robot_description_timeout_s").value)
        self.enable_reachability_gate = bool(self.get_parameter("enable_reachability_gate").value)
        self.reachability_map_file = self.get_parameter(
            "reachability_map_file"
        ).get_parameter_value().string_value
        self.reachability_query_radius = float(self.get_parameter("reachability_query_radius").value)
        self.reachability_min_neighbors = int(self.get_parameter("reachability_min_neighbors").value)
        self.reachability_snap_rejected_marker = bool(
            self.get_parameter("reachability_snap_rejected_marker").value
        )
        self.arm_controlled = self._normalize_arm_controlled(
            self.get_parameter("arm_controlled").get_parameter_value().string_value
        )
        self.control_right_arm = self.arm_controlled in ("right", "both")
        self.control_left_arm = self.arm_controlled in ("left", "both")
        self.controlled_arm_joints = self._controlled_arm_joints(self.arm_controlled)

        self.control_dt = 0.005
        self.time = 0.0
        self.t = 0.0
        self.init_duration_s = 3.0

        self.mode = Mode.PR
        self.mode_machine = 0
        self.motors_on = 1

        self.right_hand_goal = None
        self.left_hand_goal = None
        self.emergency_stop = False
        self._initialized = False
        self.arms_enabled = False
        self.reachability_map = None
        self.last_accepted_goals = {}
        self.rejected_marker_snaps = set()
        self._last_reachability_warning_time = {}

        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.base_height_publisher = self.create_publisher(Float64, "/base_height", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.arms_enabled_sub = self.create_subscription(Bool, ARMS_ENABLED_TOPIC, self.arms_enabled_callback, 10)
        self.arms_home_sub = self.create_subscription(Bool, ARMS_HOME_TOPIC, self.arms_home_callback, 10)
        self.emergency_stop_sub = self.create_subscription(Bool, "/g1pilot/emergency_stop", self.emergency_stop_callback, 10)
        if self.control_right_arm:
            self.right_hand_goal_subscriber = self.create_subscription(
                PoseStamped, RIGHT_HAND_GOAL_TOPIC, self.right_hand_goal_callback, 10
            )
        if self.control_left_arm:
            self.left_hand_goal_subscriber = self.create_subscription(
                PoseStamped, LEFT_HAND_GOAL_TOPIC, self.left_hand_goal_callback, 10
            )

        self.urdf = robot_description_param.strip()
        if self.urdf:
            self.get_logger().info("Using robot_description parameter for OpenSoT model")
        else:
            self.client = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
            wait_started = time.monotonic()
            while not self.client.wait_for_service(timeout_sec=1.0):
                elapsed = time.monotonic() - wait_started
                if elapsed >= self.robot_description_timeout_s:
                    raise RuntimeError(
                        "Timed out waiting for /robot_state_publisher/get_parameters. "
                        "Pass robot_description directly or start robot_state_publisher."
                    )
                self.get_logger().warn("Service /robot_state_publisher/get_parameters not available, waiting...")

            request = GetParameters.Request()
            request.names = ["robot_description"]
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.robot_description_timeout_s)

            if future.done() and future.result() is not None:
                values = future.result().values
                for val in values:
                    self.urdf = val.string_value
            else:
                raise RuntimeError("Failed to get robot_description from robot_state_publisher")

        self._initialize_reachability_gate()

        self.interactive_marker_server = InteractiveMarkerServer(self, "teleoperation_markers")
        self.marker_poses = {}
        self.marker_enabled = {}
        self.menu_handler = {}
        self.menu_entry_ids = {}

        self.collision_distances_publisher = self.create_publisher(Marker, 'collision_distances', 10)

        self.right_hand_frame_ref = "pelvis"
        self.left_hand_frame_ref = "pelvis"

        self.motor_state = [MotorState() for _ in range(35)]
        self.lowstate_buffer = DataBuffer()

        self.lowcmd_publisher = None
        self.lowstate_subscriber = None
        self.subscribe_thread = None
        self.crc = None
        self.msg = None
        self.all_motor_q = None
        self.model_joint_names = []
        self.model_q_index_by_motor_id = {}
        self._missing_model_motor_warnings = set()

        if self.publish_arm_sdk:
            if not self.use_robot:
                self.get_logger().warn(
                    "publish_arm_sdk=True with use_robot=False -> publishing OpenSoT "
                    "arm commands to Unitree DDS for simulation. Ensure interface is loopback."
                )
            else:
                self.get_logger().info("use_robot=True -> Initializing Unitree DDS interface")
            self.initialize_interface()
        else:
            self.get_logger().warn(
                "use_robot=False and publish_arm_sdk=False -> running in "
                "visualization-only mode (publishing only ROS state topics)."
            )
        if self.publish_pelvis_tf:
            self.get_logger().info("OpenSoT will publish world -> pelvis TF.")
        else:
            self.get_logger().info("OpenSoT will not publish pelvis TF.")

        self.initialize()
        self.initialize_imarkers()

        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

    def _subscribe_motor_state(self):
        while rclpy.ok():
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                self.lowstate_buffer.SetData(msg)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q = msg.motor_state[i].q
                    self.motor_state[i].dq = msg.motor_state[i].dq
            time.sleep(0.001)

    def _normalize_arm_controlled(self, arm_controlled):
        arm_controlled = arm_controlled.strip().lower()
        if arm_controlled not in ("left", "right", "both"):
            raise ValueError(
                "arm_controlled must be one of: left, right, both "
                f"(got {arm_controlled!r})"
            )
        return arm_controlled

    def _controlled_arm_joints(self, arm_controlled):
        if arm_controlled == "left":
            return LEFT_ARM_JOINTS
        if arm_controlled == "right":
            return RIGHT_ARM_JOINTS
        return LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

    def initialize_interface(self):
        ChannelFactoryInitialize(self.domain_id, self.interface)

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init()

        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state, daemon=True)
        self.subscribe_thread.start()

        self.lowcmd_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.lowcmd_publisher.Init()

        while not self.lowstate_buffer.GetData():
            self.get_logger().info("Waiting for LowState data...")
            time.sleep(0.01)

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()

        self.kp_high = 300.0
        self.kd_high = 3.0
        self.kp_low = 150.0
        self.kd_low = 4.0
        self.kp_wrist = 40.0
        self.kd_wrist = 1.5

        wrist_vals = {m.value for m in G1_29_JointWristIndex}
        for jid in G1_29_JointArmIndex:
            self.msg.motor_cmd[jid].mode = 1 if jid in self.controlled_arm_joints else 0
            if jid.value in wrist_vals:
                self.msg.motor_cmd[jid].kp = self.kp_wrist
                self.msg.motor_cmd[jid].kd = self.kd_wrist
            else:
                self.msg.motor_cmd[jid].kp = self.kp_low
                self.msg.motor_cmd[jid].kd = self.kd_low
            self.msg.motor_cmd[jid].q = float(self.all_motor_q[jid.value])

        self._initialized = True

    def get_mode_machine(self) -> int:
        msg = self.lowstate_buffer.GetData()
        return msg.mode_machine

    def get_current_motor_q(self):
        msg = self.lowstate_buffer.GetData()
        if msg is None:
            return np.zeros(29)
        q = np.zeros(29)
        for i in range(29):
            q[i] = msg.motor_state[i].q
        return q

    def _initialize_reachability_gate(self):
        if not self.enable_reachability_gate:
            self.get_logger().info("Reachability gate disabled.")
            return

        if not self.reachability_map_file:
            raise RuntimeError(
                "enable_reachability_gate:=true requires reachability_map_file."
            )

        self.reachability_map = ReachabilityMap.load(self.reachability_map_file)
        self.get_logger().info(
            "Reachability gate enabled: "
            f"map={self.reachability_map_file}, "
            f"radius={self.reachability_query_radius:.3f} m, "
            f"min_neighbors={self.reachability_min_neighbors}"
        )
        self.get_logger().info("\n" + self.reachability_map.summary())

    def _pose_stamped_to_affine(self, ps):
        T = pyaffine3.Affine3()
        T.translation = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
        T.linear = R.from_quat([
            ps.pose.orientation.x,
            ps.pose.orientation.y,
            ps.pose.orientation.z,
            ps.pose.orientation.w,
        ]).as_matrix()
        return T

    def _reachability_accepts(self, side, ps):
        if self.reachability_map is None:
            return True

        target = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
        result = self.reachability_map.query(
            side,
            target,
            radius=self.reachability_query_radius,
            min_neighbors=self.reachability_min_neighbors,
        )
        if result.reachable:
            self.last_accepted_goals[side] = deepcopy(ps)
            return True

        now = time.monotonic()
        last_warn = self._last_reachability_warning_time.get(side, 0.0)
        if now - last_warn > 1.0:
            nearest = "none"
            if result.nearest_position is not None:
                nearest = (
                    f"({result.nearest_position[0]:.3f}, "
                    f"{result.nearest_position[1]:.3f}, "
                    f"{result.nearest_position[2]:.3f})"
                )
            self.get_logger().warn(
                f"Rejected {side} hand target "
                f"({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) "
                f"outside reachability map: neighbors={result.neighbors}, "
                f"nearest_distance={result.nearest_distance:.3f} m, nearest={nearest}"
            )
            self._last_reachability_warning_time[side] = now
        return False

    def _maybe_snap_rejected_marker(self, marker_name, side):
        if not self.reachability_snap_rejected_marker:
            return
        last_goal = self.last_accepted_goals.get(side)
        if last_goal is None:
            return
        self.marker_poses[marker_name] = deepcopy(last_goal)
        self.interactive_marker_server.setPose(marker_name, last_goal.pose, last_goal.header)
        self.interactive_marker_server.applyChanges()

    def _set_gripper_reference_if_reachable(self, side, gripper, ps, marker_name=None):
        if not self._reachability_accepts(side, ps):
            if marker_name is not None:
                self.rejected_marker_snaps.add(marker_name)
            return False
        gripper.setReference(self._pose_stamped_to_affine(ps))
        return True

    def _marker_side(self, marker_name):
        if marker_name.startswith("right_"):
            return "right"
        if marker_name.startswith("left_"):
            return "left"
        return None

    def _feedback_pose_stamped(self, feedback):
        ps = PoseStamped()
        ps.header = feedback.header
        ps.pose = feedback.pose
        return ps

    def _configure_model_joint_mapping(self):
        self.model_joint_names = list(self.model.getJointNames()[1:])
        model_q_index_by_joint_name = {
            joint_name: 7 + idx for idx, joint_name in enumerate(self.model_joint_names)
        }
        self.model_q_index_by_motor_id = {
            motor_id: model_q_index_by_joint_name[joint_name]
            for motor_id, joint_name in JOINT_NAMES_ROS.items()
            if joint_name in model_q_index_by_joint_name
        }

        missing_core_joints = [
            joint_name
            for motor_id, joint_name in JOINT_NAMES_ROS.items()
            if motor_id in range(G1_NUM_MOTOR)
            and joint_name not in model_q_index_by_joint_name
        ]
        if missing_core_joints:
            self.get_logger().warn(
                "Model is missing some G1 motor joints; they will not be commanded: "
                + ", ".join(missing_core_joints)
            )

    def _initial_model_q(self):
        q = np.zeros(self.model.nq)
        q[2] = 0.6756
        q[6] = 1.0

        motor_q = None
        if self.publish_arm_sdk and self.all_motor_q is not None:
            motor_q = np.asarray(self.all_motor_q, dtype=float)

        initialized_from_lowstate = 0
        initialized_from_default = 0
        for motor_id, q_index in self.model_q_index_by_motor_id.items():
            if motor_q is not None and motor_id < len(motor_q):
                q[q_index] = float(motor_q[motor_id])
                initialized_from_lowstate += 1
            elif motor_id < len(q_init):
                q[q_index] = float(q_init[motor_id])
                initialized_from_default += 1

        unmapped_model_joints = len(self.model_joint_names) - len(self.model_q_index_by_motor_id)
        if motor_q is not None:
            self.get_logger().info(
                f"Initialized {initialized_from_lowstate} model joints from LowState motor positions."
            )
        if initialized_from_default:
            self.get_logger().info(
                f"Initialized {initialized_from_default} model joints from q_init defaults."
            )
        if unmapped_model_joints:
            self.get_logger().warn(
                f"{unmapped_model_joints} model joints have no G1 motor-index mapping; leaving them at zero."
            )

        return q

    def _model_q_for_motor(self, motor_id):
        motor_id = int(motor_id)
        q_index = self.model_q_index_by_motor_id.get(motor_id)
        if q_index is None:
            if motor_id not in self._missing_model_motor_warnings:
                self._missing_model_motor_warnings.add(motor_id)
                joint_name = JOINT_NAMES_ROS.get(motor_id, f"motor_{motor_id}")
                self.get_logger().warn(
                    f"Skipping command for {joint_name}: joint is not active in the selected model."
                )
            return None
        return float(self.q[q_index])

    def right_hand_goal_callback(self, msg: PoseStamped):
        self.right_hand_goal = msg

    def left_hand_goal_callback(self, msg: PoseStamped):
        self.left_hand_goal = msg

    def arms_enabled_callback(self, msg: Bool):
        self.set_arm_control_enabled(bool(msg.data), ARMS_ENABLED_TOPIC)

    def set_arm_control_enabled(self, enabled, source):
        self.arms_enabled = enabled
        state = "enabled" if enabled else "disabled"
        self.get_logger().info(f"OpenSoT arm control {state} by {source}")

    def arms_home_callback(self, msg: Bool):
        if not msg.data:
            return
        self.reset_hand_goals_to_home()

    def emergency_stop_callback(self, msg: Bool):
        self.emergency_stop = bool(msg.data)

    def reset_hand_goals_to_home(self):
        reset_any = False
        if self.control_right_arm:
            reset_any = self.reset_marker_to_home("right_hand_marker", "right") or reset_any
        if self.control_left_arm:
            reset_any = self.reset_marker_to_home("left_hand_marker", "left") or reset_any
        if reset_any:
            self.interactive_marker_server.applyChanges()

    def reset_marker_to_home(self, marker_name, side):
        home = self.marker_home_poses.get(marker_name, None)
        if home is None:
            self.get_logger().warn(f"Cannot home {side} hand: no marker home pose stored.")
            return False

        home_goal = deepcopy(home)
        home_goal.header.stamp = self.get_clock().now().to_msg()
        self.marker_poses[marker_name] = deepcopy(home_goal)

        if side == "right":
            self.right_hand_goal = deepcopy(home_goal)
        elif side == "left":
            self.left_hand_goal = deepcopy(home_goal)
        self.last_accepted_goals[side] = deepcopy(home_goal)

        self.interactive_marker_server.setPose(marker_name, home_goal.pose, home_goal.header)
        self.get_logger().info(f"Reset {side} hand goal to home.")
        return True

    def initialize_imarkers(self):
        self.marker_enabled = {}
        self.menu_handler = {}

        base_ref, _ = self.base.getReference()
        com_ref, _ = self.com.getReference()
        pose_ref = pyaffine3.Affine3()
        pose_ref.translation = com_ref
        pose_ref.linear = base_ref.linear.copy()
        # self.get_logger().info(f"Initial base pose:\n{pose_ref}")
        # self.make_6dof_marker("base_marker", pose_ref, "world")

        if self.control_right_arm:
            right_hand_ref = self.right_gripper.getReference()
            self.get_logger().info(f"Initial right hand pose:\n{right_hand_ref}")
            self.make_6dof_marker("right_hand_marker", right_hand_ref[0], self.right_hand_frame_ref)

        if self.control_left_arm:
            left_hand_ref = self.left_gripper.getReference()
            self.get_logger().info(f"Initial left hand pose:\n{left_hand_ref}")
            self.make_6dof_marker("left_hand_marker", left_hand_ref[0], self.left_hand_frame_ref)

    def make_6dof_marker(self, name, pose, frame_id):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = name
        int_marker.description = '6-DOF Control'
        int_marker.scale = 0.3

        int_marker.pose.position.x = pose.translation[0]
        int_marker.pose.position.y = pose.translation[1]
        int_marker.pose.position.z = pose.translation[2]

        quat_xyzw = R.from_matrix(pose.linear).as_quat() # Format: [x, y, z, w]
        int_marker.pose.orientation.x = quat_xyzw[0]
        int_marker.pose.orientation.y = quat_xyzw[1]
        int_marker.pose.orientation.z = quat_xyzw[2]
        int_marker.pose.orientation.w = quat_xyzw[3]

        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose = int_marker.pose
        self.marker_poses[name] = ps
        if name.startswith("right_"):
            self.last_accepted_goals["right"] = deepcopy(ps)
        elif name.startswith("left_"):
            self.last_accepted_goals["left"] = deepcopy(ps)

        self.marker_home_poses = getattr(self, "marker_home_poses", {})
        self.marker_home_poses[name] = PoseStamped()
        self.marker_home_poses[name].header.frame_id = frame_id
        self.marker_home_poses[name].pose = int_marker.pose

        # Add a visible marker (e.g., a cube)
        cube_marker = Marker()
        cube_marker.type = Marker.CUBE
        cube_marker.scale.x = 0.05
        cube_marker.scale.y = 0.05
        cube_marker.scale.z = 0.05
        cube_marker.color.r = 0.0
        cube_marker.color.g = 1.0
        cube_marker.color.b = 0.0
        cube_marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(cube_marker)
        int_marker.controls.append(control)

        # Add 6-DOF controls
        self.add_6dof_controls(int_marker)

        self.marker_enabled[name] = False

        menu = MenuHandler()
        h_enable = menu.insert("Enable", callback=self.process_menu)
        menu.setCheckState(
            h_enable,
            MenuHandler.CHECKED if self.marker_enabled.get(name, True) else MenuHandler.UNCHECKED
        )
        h_reset = menu.insert("Reset", callback=self.process_menu)
        self.menu_handler[name] = menu

        self.menu_entry_ids = getattr(self, "menu_entry_ids", {})
        self.menu_entry_ids[name] = {"enable": h_enable, "reset": h_reset}

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.name = "menu"
        int_marker.controls.append(menu_control)

        self.interactive_marker_server.insert(marker=int_marker, feedback_callback=self.process_feedback)
        menu.apply(self.interactive_marker_server, name)
        self.interactive_marker_server.applyChanges()

    def process_feedback(self, feedback):
        name = feedback.marker_name
        if not self.marker_enabled.get(name, True):
            return

        side = self._marker_side(name)
        ps = self._feedback_pose_stamped(feedback)
        if side is not None and self.reachability_map is not None:
            if not self._reachability_accepts(side, ps):
                self.rejected_marker_snaps.add(name)
                if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                    self._maybe_snap_rejected_marker(name, side)
                    self.rejected_marker_snaps.discard(name)
                return
            self.rejected_marker_snaps.discard(name)

        self.marker_poses[name] = ps

        if (
            feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP
            and name in self.rejected_marker_snaps
            and side is not None
        ):
            self._maybe_snap_rejected_marker(name, side)
            self.rejected_marker_snaps.discard(name)

    def process_menu(self, feedback):
        name = feedback.marker_name
        ids = self.menu_entry_ids.get(name, {})

        if feedback.menu_entry_id == ids.get("enable"):
            new_state = not self.marker_enabled.get(name, True)
            self.marker_enabled[name] = new_state

            menu = self.menu_handler.get(name, None)
            if menu is not None:
                menu.setCheckState(
                    ids["enable"],
                    MenuHandler.CHECKED if new_state else MenuHandler.UNCHECKED
                )
                menu.reApply(self.interactive_marker_server)
                self.interactive_marker_server.applyChanges()


        elif feedback.menu_entry_id == ids.get("reset"):
            side = "right" if name.startswith("right_") else "left"
            if self.reset_marker_to_home(name, side):
                self.interactive_marker_server.applyChanges()


    def add_6dof_controls(self, marker):
        axes = ['x', 'y', 'z']
        for axis in axes:
            # Rotation
            control = InteractiveMarkerControl()
            control.name = f'rotate_{axis}'
            control.orientation.w = 1.0
            setattr(control.orientation, axis, 1.0)
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            marker.controls.append(control)

            # Translation
            control = InteractiveMarkerControl()
            control.name = f'move_{axis}'
            control.orientation.w = 1.0
            setattr(control.orientation, axis, 1.0)
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            marker.controls.append(control)

    # ----------------------------
    # OpenSoT init
    # ----------------------------
    def initialize(self, manipulation_frame="world"):
        self.get_logger().warning("Initializing XBot2 Model Interface")

        if not self.urdf or len(self.urdf.strip()) < 100:
            self.get_logger().error(f"robot_description invalid. len={0 if not self.urdf else len(self.urdf)}")
            raise RuntimeError("robot_description is empty/invalid -> cannot build ModelInterface2")


        self.model = xbi.ModelInterface2(self.urdf)
        self._configure_model_joint_mapping()

        self.q = self._initial_model_q()

        self.dq = np.zeros(self.model.nv)

        self.model.setJointPosition(self.q)
        self.model.setJointVelocity(self.dq)
        self.model.update()

        self.com = CoM(self.model)

        self.get_logger().warning("Initializing OpenSoT Tasks and Constraints")

        self.get_logger().info("Task: Base")
        self.base = Cartesian("base_task", self.model, "pelvis", manipulation_frame)
        self.base.setLambda(0.1)

        self.get_logger().info("Task: Torso")
        self.torso = Cartesian("torso_task", self.model, "torso_link", manipulation_frame)
        self.torso.setLambda(0.1)

        self.get_logger().info("Task: Right Gripper")
        self.right_gripper = Cartesian(
            "right_gripper_task",
            self.model,
            "right_hand_point_contact",
            self.right_hand_frame_ref,
        )
        self.right_gripper.setLambda(0.1)

        self.get_logger().info("Task: Left Gripper")
        self.left_gripper = Cartesian(
            "left_gripper_task",
            self.model,
            "left_hand_point_contact",
            self.left_hand_frame_ref,
        )
        self.left_gripper.setLambda(0.1)

        self.get_logger().info("Task: Postural")
        self.postural = Postural(self.model)
        self.postural.setLambda(0.1)
        self.W = self.postural.getWeight().copy()
        #W[0:6, 0:6] = 0.0
        #W[6:10, 6:10] = 0.0
        print(self.W.shape)
        self.postural.setWeight(self.W)

        print(self.model.getNv())
        print(self.model.getNq())

        self.get_logger().info("Constraints: Joint Limits")
        self.qmin, self.qmax = self.model.getJointLimits()
        self.qlims = JointLimits(self.model, self.qmax, self.qmin)
        print(self.qmin)
        self.dqmax = self.model.getVelocityLimits()
        self.dqlims = VelocityLimits(self.model, self.dqmax, self.control_dt)

        self.collision_avoidance_constraint = None
        if self.enable_collision_avoidance:
            if CollisionAvoidance is None:
                raise RuntimeError(
                    "enable_collision_avoidance:=true requires pyopensot_collision, "
                    "but that module is not available."
                )

            self.get_logger().info("Constraints: Self-Collision Avoidance")
            self.collision_avoidance_constraint = CollisionAvoidance(
                self.model, max_pairs=50, collision_urdf=self.urdf)#, collision_srdf=self.urdf)

            # All arm links and torso now have primitive collision geometries in g1_29dof.urdf.
            collision_list = {
                # Left arm vs torso
                ("left_shoulder_yaw_link", "torso_link"),
                ("left_elbow_link", "torso_link"),
                ("left_wrist_roll_link", "torso_link"),
                ("left_wrist_pitch_link", "torso_link"),
                ("left_wrist_yaw_link", "torso_link"),
                ("left_rubber_hand", "torso_link"),
                # Right arm vs torso
                ("right_shoulder_yaw_link", "torso_link"),
                ("right_elbow_link", "torso_link"),
                ("right_wrist_roll_link", "torso_link"),
                ("right_wrist_pitch_link", "torso_link"),
                ("right_wrist_yaw_link", "torso_link"),
                ("right_rubber_hand", "torso_link"),
                # hip
                ("left_rubber_hand", "waist_yaw_link"),
                ("right_rubber_hand", "waist_yaw_link"),
                # pelvis
                ("left_rubber_hand", "pelvis_contour_link"),
                ("right_rubber_hand", "pelvis_contour_link"),
                # Left hand vs legs
                ("left_rubber_hand", "left_hip_pitch_link"),
                ("left_rubber_hand", "left_hip_roll_link"),
                ("left_rubber_hand", "left_hip_yaw_link"),
                ("left_rubber_hand", "left_knee_link"),
                ("left_rubber_hand", "right_hip_pitch_link"),
                ("left_rubber_hand", "right_hip_roll_link"),
                ("left_rubber_hand", "right_hip_yaw_link"),
                ("left_rubber_hand", "right_knee_link"),
                # Right hand vs legs
                ("right_rubber_hand", "left_hip_pitch_link"),
                ("right_rubber_hand", "left_hip_roll_link"),
                ("right_rubber_hand", "left_hip_yaw_link"),
                ("right_rubber_hand", "left_knee_link"),
                ("right_rubber_hand", "right_hip_pitch_link"),
                ("right_rubber_hand", "right_hip_roll_link"),
                ("right_rubber_hand", "right_hip_yaw_link"),
                ("right_rubber_hand", "right_knee_link"),
            }

            self.collision_avoidance_constraint.setCollisionList(collision_list)
            self.collision_avoidance_constraint.setBoundScaling(0.1)
            self.collision_avoidance_constraint.setLinkPairThreshold(0.01)
            self.collision_avoidance_constraint.setDetectionThreshold(-1)
        

        # self.com_xy = self.com % [0, 1]
        # self.stack = (
        #     self.com_xy
        #     / (self.base % [3, 4, 5] + self.torso % [3, 4, 5] + self.right_gripper + self.left_gripper)
        #     / self.postural
        #     << self.qlims
        #     << self.dqlims
        # )
        torso_and_hands = self.torso % [3, 4, 5]
        if self.control_right_arm:
            torso_and_hands = torso_and_hands + self.right_gripper
        if self.control_left_arm:
            torso_and_hands = torso_and_hands + self.left_gripper

        self.get_logger().info(f"OpenSoT arm task selection: {self.arm_controlled}")

        self.stack = ((
            self.base#%[0,1,3,4,5]
            / torso_and_hands
            / self.postural)
            << self.qlims
            << self.dqlims
        )

        if self.collision_avoidance_constraint is not None:
            self.stack = self.stack << self.collision_avoidance_constraint
            
        self.stack.update()
        self.solver = pysot.iHQP(self.stack, eps_regularisation=1e11)

    # ----------------------------
    # Control loop
    # ----------------------------
    def _send_passive_arm_command(self):
        if not self.publish_arm_sdk:
            return
        if self.lowcmd_publisher is None or self.msg is None or self.crc is None:
            return

        wrist_vals = {m.value for m in G1_29_JointWristIndex}
        for jid in G1_29_JointArmIndex:
            self.msg.motor_cmd[jid].mode = 0
            if jid.value in wrist_vals:
                self.msg.motor_cmd[jid].kp = self.kp_wrist
                self.msg.motor_cmd[jid].kd = self.kd_wrist
            else:
                self.msg.motor_cmd[jid].kp = self.kp_low
                self.msg.motor_cmd[jid].kd = self.kd_low

            self.msg.motor_cmd[jid].q = float(self.msg.motor_cmd[jid].q)

        self.msg.crc = self.crc.Crc(self.msg)
        if self.send_cmds_to_robot:
            self.lowcmd_publisher.Write(self.msg)

    def control_loop(self):
        if self.emergency_stop:
            self._send_passive_arm_command()
            return

        if not self.arms_enabled:
            return

        wrist_vals = {m.value for m in G1_29_JointWristIndex}

        self.model.setJointPosition(self.q)
        self.model.setJointVelocity(self.dq)
        self.model.update()

        if self.control_right_arm:
            use_marker_right = self.marker_enabled.get("right_hand_marker", False) and "right_hand_marker" in self.marker_poses
            if use_marker_right:
                ps = self.marker_poses["right_hand_marker"]
                self._set_gripper_reference_if_reachable(
                    "right", self.right_gripper, ps, marker_name="right_hand_marker"
                )
            elif self.right_hand_goal is not None:
                ps = self.right_hand_goal
                self._set_gripper_reference_if_reachable("right", self.right_gripper, ps)

        if self.control_left_arm:
            use_marker_left = self.marker_enabled.get("left_hand_marker", False) and "left_hand_marker" in self.marker_poses
            if use_marker_left:
                ps = self.marker_poses["left_hand_marker"]
                self._set_gripper_reference_if_reachable(
                    "left", self.left_gripper, ps, marker_name="left_hand_marker"
                )
            elif self.left_hand_goal is not None:
                ps = self.left_hand_goal
                self._set_gripper_reference_if_reachable("left", self.left_gripper, ps)

        # solve
        self.stack.update()
        try:
            dq = self.solver.solve()
            self.q = self.model.sum(self.q, dq)
            self.dq = dq

        except Exception as e:
            self.get_logger().error(f"OpenSoT Solver Error: {e}")
            dq = None

        if self.publish_pelvis_tf:
            self._publish_pelvis_tf()


        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()


        try:
            js.name = self.model.getJointNames()[1::]
            js.position = self.q[7:].tolist()
        except Exception:
            js.name = []
            js.position = []
            self.get_logger().error("Error getting joint names from model")
        if self.publish_joint_states_opensot:
            self.joint_state_publisher.publish(js)

        msg = Float64()
        msg.data = self.q[2]

        self.base_height_publisher.publish(msg)

        if self.collision_avoidance_constraint is not None:
            self.publishCollisionDistances(
                self.collision_avoidance_constraint.getOrderedWitnessPointVector(),
                self.get_clock().now().to_msg(),
            )

        if not self.publish_arm_sdk:
            return

        if self.lowcmd_publisher is None or self.msg is None or self.crc is None:
            return

        if not self.motors_on:
            self._send_passive_arm_command()
            return

        for jid in self.controlled_arm_joints:
            q_cmd = self._model_q_for_motor(jid.value)
            if q_cmd is None:
                self.msg.motor_cmd[jid].mode = 0
                continue

            self.msg.mode_machine = self.get_mode_machine()
            self.msg.motor_cmd[jid].mode = 1
            if jid.value in wrist_vals:
                self.msg.motor_cmd[jid].kp = self.kp_wrist
                self.msg.motor_cmd[jid].kd = self.kd_wrist
            else:
                self.msg.motor_cmd[jid].kp = self.kp_low
                self.msg.motor_cmd[jid].kd = self.kd_low

            self.msg.motor_cmd[jid].q = q_cmd

        for wid in G1_29_JointWaistIndex:
            q_cmd = self._model_q_for_motor(wid.value)
            if q_cmd is None:
                self.msg.motor_cmd[wid].mode = 0
                continue

            self.msg.motor_cmd[wid].mode = 1
            self.msg.motor_cmd[wid].kp = self.kp_low
            self.msg.motor_cmd[wid].kd = self.kd_low
            self.msg.motor_cmd[wid].dq = 0.0
            self.msg.motor_cmd[wid].tau = 0.0

            self.msg.motor_cmd[wid].q = q_cmd

        try:
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
        except Exception:
            pass

        self.msg.mode_pr = 1
        self.msg.crc = self.crc.Crc(self.msg)

        if self.send_cmds_to_robot:
            self.lowcmd_publisher.Write(self.msg)

    def _publish_pelvis_tf(self):
        t = TransformStamped()
        t.header.frame_id = "world"
        t.child_frame_id = "pelvis"
        t.header.stamp = self.get_clock().now().to_msg()
        t.transform.translation.x = self.q[0]
        t.transform.translation.y = self.q[1]
        t.transform.translation.z = self.q[2]
        t.transform.rotation.x = self.q[3]
        t.transform.rotation.y = self.q[4]
        t.transform.rotation.z = self.q[5]
        t.transform.rotation.w = self.q[6]

        self.tf_broadcaster.sendTransform(t)

    def publishCollisionDistances(self, collision_distance_points, time):
        marker = Marker()
        marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0
        marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.header.frame_id = "world"
        marker.header.stamp = time
        marker.ns = "collision_distances"
        marker.id = 0
        marker.scale.x = 0.005  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Opaque

        for point_pairs in collision_distance_points:
            pa = point_pairs[0]
            pb = point_pairs[1]

            point_a = Point()
            point_a.x = pa[0]
            point_a.y = pa[1]
            point_a.z = pa[2]

            point_b = Point()
            point_b.x = pb[0]
            point_b.y = pb[1]
            point_b.z = pb[2]

            marker.points.append(point_a)
            marker.points.append(point_b)


        self.collision_distances_publisher.publish(marker)
        


def main(args=None):
    rclpy.init(args=args)
    node = None
    shutdown_exceptions = (KeyboardInterrupt, ExternalShutdownException)
    if RCLError is not None:
        shutdown_exceptions = shutdown_exceptions + (RCLError,)

    try:
        node = G1CollisionAvoidanceNode()
        rclpy.spin(node)
    except shutdown_exceptions:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

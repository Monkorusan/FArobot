from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from ament_index_python.packages import get_package_share_directory

try:
    from moveit.planning import MoveItPy, PlanningComponent, PlanRequestParameters
except ImportError:  # pragma: no cover - handled at runtime
    MoveItPy = None
    PlanningComponent = None
    PlanRequestParameters = None

from FArobot.packing_config import PackingConfig, load_config


@dataclass(frozen=True)
class Offsets:
    approach: float
    lift: float


class PickPlaceExecutor(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_executor_node")

        share_dir = get_package_share_directory("FArobot")
        default_yaml = os.path.join(share_dir, "config", "boxes.yaml")

        self.declare_parameter("config", default_yaml)
        self.declare_parameter("planning_group", "ur_manipulator")
        self.declare_parameter("eef_link", "tool0")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("planning_pipeline", "ompl")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("planning_time", 2.0)
        self.declare_parameter("planning_attempts", 3)
        self.declare_parameter("max_velocity_scaling", 0.2)
        self.declare_parameter("max_acceleration_scaling", 0.2)
        self.declare_parameter("tool_orientation_rpy", [3.1416, 0.0, 0.0])
        self.declare_parameter("min_goal_z", 0.1)
        self.declare_parameter("execution_mode", "simulate")
        self.declare_parameter("max_boxes", 1)
        self.declare_parameter(
            "joint_names",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        self.declare_parameter("enable_attach", True)
        self.declare_parameter("approach_offset", 0.08)
        self.declare_parameter("lift_offset", 0.12)
        self.declare_parameter("retry_offsets", [0.0, 0.05, 0.1])
        self.declare_parameter("auto_start", True)

        yaml_path = self.get_parameter("config").get_parameter_value().string_value
        self._config: PackingConfig = load_config(yaml_path)

        self._planning_group = self.get_parameter("planning_group").get_parameter_value().string_value
        self._eef_link = self.get_parameter("eef_link").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self._planning_pipeline = (
            self.get_parameter("planning_pipeline").get_parameter_value().string_value
        )
        self._planner_id = self.get_parameter("planner_id").get_parameter_value().string_value
        self._planning_time = self.get_parameter("planning_time").get_parameter_value().double_value
        self._planning_attempts = (
            self.get_parameter("planning_attempts").get_parameter_value().integer_value
        )
        self._max_velocity_scaling = (
            self.get_parameter("max_velocity_scaling").get_parameter_value().double_value
        )
        self._max_acceleration_scaling = (
            self.get_parameter("max_acceleration_scaling").get_parameter_value().double_value
        )
        self._tool_orientation_rpy = list(
            self.get_parameter("tool_orientation_rpy").get_parameter_value().double_array_value
        )
        self._min_goal_z = self.get_parameter("min_goal_z").get_parameter_value().double_value
        self._execution_mode = self.get_parameter("execution_mode").get_parameter_value().string_value
        self._max_boxes = self.get_parameter("max_boxes").get_parameter_value().integer_value
        self._joint_names = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self._enable_attach = self.get_parameter("enable_attach").get_parameter_value().bool_value

        offsets = Offsets(
            approach=self.get_parameter("approach_offset").get_parameter_value().double_value,
            lift=self.get_parameter("lift_offset").get_parameter_value().double_value,
        )
        self._offsets = offsets
        self._auto_start = self.get_parameter("auto_start").get_parameter_value().bool_value
        self._retry_offsets = list(
            self.get_parameter("retry_offsets").get_parameter_value().double_array_value
        )

        self._pick_poses: List[Pose] = []
        self._place_poses: List[Pose] = []
        self._started = False
        self._scene_ready = False
        self._box_ids: List[str] = [box.box_id for box in self._config.boxes]
        self._box_sizes: Dict[str, tuple[float, float, float]] = {
            box.box_id: (box.size_x, box.size_y, box.size_z) for box in self._config.boxes
        }

        self.create_subscription(PoseArray, "fa_robot/pick_poses", self._on_pick_poses, 10)
        self.create_subscription(PoseArray, "fa_robot/place_poses", self._on_place_poses, 10)

        self._planner: Optional[PlanningComponent] = None
        self._moveit = None
        self._plan_request_parameters = None
        self._display_pub = self.create_publisher(DisplayTrajectory, "display_planned_path", 10)
        self._joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self._publish_initial_joint_state()
        if MoveItPy is None or PlanningComponent is None:
            self.get_logger().error(
                "MoveItPy is not available. Install moveit_py and source the setup.bash."
            )
        else:
            self._moveit = MoveItPy(node_name="fa_pick_place")
            self._planner = PlanningComponent(self._planning_group, self._moveit)
            if PlanRequestParameters is not None:
                params = PlanRequestParameters(self._moveit)
                params.planning_pipeline = self._planning_pipeline
                params.planner_id = self._planner_id
                params.planning_time = self._planning_time
                params.planning_attempts = int(self._planning_attempts)
                params.max_velocity_scaling_factor = self._max_velocity_scaling
                params.max_acceleration_scaling_factor = self._max_acceleration_scaling
                self._plan_request_parameters = params

        self._planning_scene_pub = self.create_publisher(PlanningScene, "planning_scene", 10)
        self._timer = self.create_timer(0.5, self._tick)

    def _on_pick_poses(self, msg: PoseArray) -> None:
        self._pick_poses = list(msg.poses)

    def _on_place_poses(self, msg: PoseArray) -> None:
        self._place_poses = list(msg.poses)

    def _tick(self) -> None:
        if not self._auto_start or self._started:
            return
        if not self._pick_poses or not self._place_poses:
            return
        if self._planner is None:
            return

        if not self._scene_ready:
            self._init_planning_scene()

        self._started = True
        count = min(len(self._pick_poses), len(self._place_poses), int(self._max_boxes))
        self.get_logger().info("Starting pick/place sequence for %d boxes" % count)

        for index in range(count):
            pick = self._apply_pose_defaults(self._pick_poses[index])
            place = self._apply_pose_defaults(self._place_poses[index])

            approach_pick = self._offset_pose(pick, self._offsets.approach)
            lift_pick = self._offset_pose(pick, self._offsets.lift)
            approach_place = self._offset_pose(place, self._offsets.approach)
            lift_place = self._offset_pose(place, self._offsets.lift)

            if not self._plan_and_execute_with_retry(approach_pick):
                break
            if not self._plan_and_execute_with_retry(pick):
                break

            if self._enable_attach and self._execution_mode != "simulate":
                self._attach_box(self._box_ids[index])

            if not self._plan_and_execute_with_retry(lift_pick):
                break
            if not self._plan_and_execute_with_retry(approach_place):
                break
            if not self._plan_and_execute_with_retry(place):
                break

            if self._enable_attach and self._execution_mode != "simulate":
                self._detach_box(self._box_ids[index], place)

            if not self._plan_and_execute_with_retry(lift_place):
                break

        self.get_logger().info("Pick/place sequence finished")

    def _plan_and_execute(self, pose: Pose) -> bool:
        if self._planner is None:
            return False

        goal = PoseStamped()
        goal.header.frame_id = self._base_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = pose

        self._planner.set_start_state_to_current_state()
        self._planner.set_goal_state(pose_stamped_msg=goal, pose_link=self._eef_link)
        if self._plan_request_parameters is not None:
            plan_result = self._planner.plan(self._plan_request_parameters)
        else:
            plan_result = self._planner.plan()
        if not plan_result:
            self.get_logger().warn("Planning failed")
            return False

        if self._execution_mode == "simulate":
            self._publish_display_trajectory(plan_result)
            self._simulate_joint_states(plan_result.trajectory)
            return True

        if self._moveit is None:
            self.get_logger().warn("MoveItPy not initialized; cannot execute")
            return False

        try:
            self._moveit.execute(self._planning_group, plan_result.trajectory, True)
            return True
        except Exception as exc:  # pragma: no cover - runtime safety
            self.get_logger().warn("Execution failed, publishing display trajectory: %s" % exc)
            self._publish_display_trajectory(plan_result)
            return False

    def _publish_display_trajectory(self, plan_result) -> None:
        display = DisplayTrajectory()
        if hasattr(plan_result, "start_state"):
            display.trajectory_start = plan_result.start_state
        try:
            traj_msg = plan_result.trajectory.get_robot_trajectory_msg()
        except Exception as exc:  # pragma: no cover - runtime safety
            self.get_logger().warn("Failed to convert trajectory message: %s" % exc)
            return
        display.trajectory = [traj_msg]
        self._display_pub.publish(display)

    def _simulate_joint_states(self, robot_trajectory) -> None:
        traj_msg = robot_trajectory.get_robot_trajectory_msg()
        joint_traj = traj_msg.joint_trajectory
        if not joint_traj.joint_names or not joint_traj.points:
            self.get_logger().warn("Empty trajectory; skipping simulation")
            return

        last_time = 0.0
        for point in joint_traj.points:
            target_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            sleep_time = max(0.0, target_time - last_time)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(joint_traj.joint_names)
            msg.position = list(point.positions)
            msg.velocity = list(point.velocities) if point.velocities else []
            msg.effort = list(point.effort) if point.effort else []
            self._joint_state_pub.publish(msg)

            last_time = target_time

    def _publish_initial_joint_state(self) -> None:
        if not self._joint_names:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = [0.0] * len(self._joint_names)
        self._joint_state_pub.publish(msg)

    def _plan_and_execute_with_retry(self, pose: Pose) -> bool:
        retries = self._retry_offsets if self._retry_offsets else [0.0]
        for offset in retries:
            candidate = self._offset_pose(pose, offset)
            if self._plan_and_execute(candidate):
                return True
        self.get_logger().warn("Planning failed after retries")
        return False

    @staticmethod
    def _ensure_orientation(pose: Pose) -> Pose:
        if math.isclose(pose.orientation.w, 0.0) and math.isclose(pose.orientation.x, 0.0) and math.isclose(
            pose.orientation.y, 0.0
        ) and math.isclose(pose.orientation.z, 0.0):
            pose.orientation.w = 1.0
        return pose

    def _apply_pose_defaults(self, pose: Pose) -> Pose:
        pose = self._ensure_orientation(pose)
        if math.isclose(pose.orientation.w, 1.0) and math.isclose(pose.orientation.x, 0.0) and math.isclose(
            pose.orientation.y, 0.0
        ) and math.isclose(pose.orientation.z, 0.0):
            roll, pitch, yaw = self._tool_orientation_rpy
            pose.orientation = self._rpy_to_quaternion(roll, pitch, yaw)
        if pose.position.z < self._min_goal_z:
            pose.position.z = self._min_goal_z
        return pose

    @staticmethod
    def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        quat = Quaternion()
        quat.w = cr * cp * cy + sr * sp * sy
        quat.x = sr * cp * cy - cr * sp * sy
        quat.y = cr * sp * cy + sr * cp * sy
        quat.z = cr * cp * sy - sr * sp * cy
        return quat

    @staticmethod
    def _offset_pose(pose: Pose, dz: float) -> Pose:
        offset = Pose()
        offset.position.x = pose.position.x
        offset.position.y = pose.position.y
        offset.position.z = pose.position.z + dz
        offset.orientation = pose.orientation
        return offset

    def _init_planning_scene(self) -> None:
        if not self._pick_poses:
            return

        collision_objects: List[CollisionObject] = []

        bin_object = self._make_bin_collision()
        collision_objects.append(bin_object)

        for index, box_id in enumerate(self._box_ids):
            if index >= len(self._pick_poses):
                break
            size = self._box_sizes[box_id]
            collision_objects.append(self._make_box_collision(box_id, size, self._pick_poses[index]))

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects
        self._planning_scene_pub.publish(scene)
        self._scene_ready = True

    def _make_bin_collision(self) -> CollisionObject:
        bin_spec = self._config.bin_spec
        pose = Pose()
        pose.position.x = bin_spec.origin_x + bin_spec.size_x * 0.5
        pose.position.y = bin_spec.origin_y + bin_spec.size_y * 0.5
        pose.position.z = bin_spec.origin_z + bin_spec.size_z * 0.5
        pose.orientation.w = 1.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [bin_spec.size_x, bin_spec.size_y, bin_spec.size_z]

        obj = CollisionObject()
        obj.id = "bin"
        obj.header.frame_id = self._base_frame
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def _make_box_collision(self, box_id: str, size: tuple[float, float, float], pose: Pose) -> CollisionObject:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size[0], size[1], size[2]]

        pose = self._ensure_orientation(pose)

        obj = CollisionObject()
        obj.id = box_id
        obj.header.frame_id = self._base_frame
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def _attach_box(self, box_id: str) -> None:
        size = self._box_sizes.get(box_id)
        if size is None:
            return

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size[0], size[1], size[2]]

        pose = Pose()
        pose.orientation.w = 1.0

        obj = CollisionObject()
        obj.id = box_id
        obj.header.frame_id = self._eef_link
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        attached = AttachedCollisionObject()
        attached.link_name = self._eef_link
        attached.object = obj
        attached.object.operation = CollisionObject.ADD

        remove_world = CollisionObject()
        remove_world.id = box_id
        remove_world.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [remove_world]
        scene.robot_state.attached_collision_objects = [attached]
        self._planning_scene_pub.publish(scene)

    def _detach_box(self, box_id: str, place_pose: Pose) -> None:
        size = self._box_sizes.get(box_id)
        if size is None:
            return

        attached = AttachedCollisionObject()
        attached.link_name = self._eef_link
        attached.object.id = box_id
        attached.object.operation = CollisionObject.REMOVE

        obj = self._make_box_collision(box_id, size, place_pose)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        scene.robot_state.attached_collision_objects = [attached]
        self._planning_scene_pub.publish(scene)


def main() -> None:
    rclpy.init()
    node = PickPlaceExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

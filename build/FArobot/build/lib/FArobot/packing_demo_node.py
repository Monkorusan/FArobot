from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from FArobot.packing import Bin, Box, Placement, plan_packing


@dataclass(frozen=True)
class PackingConfig:
    bin_spec: Bin
    source_spec: "SourceSpec"
    boxes: List[Box]


@dataclass(frozen=True)
class SourceSpec:
    origin_x: float
    origin_y: float
    origin_z: float
    spacing_x: float
    spacing_y: float
    cols: int
    rows: int


def load_config(yaml_path: str) -> PackingConfig:
    with open(yaml_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    bin_data = data.get("bin", {})
    bin_size = bin_data.get("size", [0.6, 0.4, 0.3])
    bin_origin = bin_data.get("origin", [0.0, 0.0, 0.0])

    bin_spec = Bin(
        size_x=float(bin_size[0]),
        size_y=float(bin_size[1]),
        size_z=float(bin_size[2]),
        origin_x=float(bin_origin[0]),
        origin_y=float(bin_origin[1]),
        origin_z=float(bin_origin[2]),
    )

    source_data = data.get("source", {})
    source_origin = source_data.get("origin", [-0.4, -0.2, 0.05])
    source_spacing = source_data.get("spacing", [0.12, 0.12])
    source_cols = int(source_data.get("cols", 3))
    source_rows = int(source_data.get("rows", 2))

    source_spec = SourceSpec(
        origin_x=float(source_origin[0]),
        origin_y=float(source_origin[1]),
        origin_z=float(source_origin[2]),
        spacing_x=float(source_spacing[0]),
        spacing_y=float(source_spacing[1]),
        cols=max(source_cols, 1),
        rows=max(source_rows, 1),
    )

    boxes: List[Box] = []
    for box in data.get("boxes", []):
        size = box.get("size", [0.1, 0.1, 0.1])
        box_id = str(box.get("id", f"box_{len(boxes):02d}"))
        boxes.append(
            Box(
                box_id=box_id,
                size_x=float(size[0]),
                size_y=float(size[1]),
                size_z=float(size[2]),
            )
        )

    return PackingConfig(bin_spec=bin_spec, source_spec=source_spec, boxes=boxes)


class PackingDemoNode(Node):
    def __init__(self) -> None:
        super().__init__("packing_demo_node")

        share_dir = get_package_share_directory("FArobot")
        default_yaml = os.path.join(share_dir, "config", "boxes.yaml")

        self.declare_parameter("config", default_yaml)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("publish_rate_hz", 1.0)

        yaml_path = self.get_parameter("config").get_parameter_value().string_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        publish_rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        self._config = load_config(yaml_path)
        self._placements, self._unplaced = plan_packing(self._config.bin_spec, self._config.boxes)
        self._pick_poses = self._compute_pick_poses(self._config.source_spec, self._config.boxes)
        self._place_poses = self._compute_place_poses(self._placements)

        self._publisher = self.create_publisher(MarkerArray, "fa_robot/packing_markers", 10)
        self._pick_pose_pub = self.create_publisher(PoseArray, "fa_robot/pick_poses", 10)
        self._place_pose_pub = self.create_publisher(PoseArray, "fa_robot/place_poses", 10)
        self._timer = self.create_timer(1.0 / max(publish_rate, 0.1), self._publish_markers)

        if self._unplaced:
            self.get_logger().warn(
                "Some boxes did not fit: %s" % ", ".join(box.box_id for box in self._unplaced)
            )

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        marker_array.markers.extend(self._create_bin_marker())
        marker_array.markers.extend(self._create_box_markers(self._placements))
        marker_array.markers.extend(self._create_pick_markers(self._pick_poses))
        self._publisher.publish(marker_array)
        self._publish_pick_place_poses()

    def _create_bin_marker(self) -> List[Marker]:
        bin_spec = self._config.bin_spec
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bin"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = bin_spec.size_x
        marker.scale.y = bin_spec.size_y
        marker.scale.z = bin_spec.size_z

        marker.pose.position.x = bin_spec.origin_x + bin_spec.size_x * 0.5
        marker.pose.position.y = bin_spec.origin_y + bin_spec.size_y * 0.5
        marker.pose.position.z = bin_spec.origin_z + bin_spec.size_z * 0.5

        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.8
        marker.color.a = 0.15

        return [marker]

    def _create_box_markers(self, placements: List[Placement]) -> List[Marker]:
        markers: List[Marker] = []
        for index, placement in enumerate(placements, start=1):
            marker = Marker()
            marker.header.frame_id = self._frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "boxes"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.scale.x = placement.size_x
            marker.scale.y = placement.size_y
            marker.scale.z = placement.size_z

            marker.pose = Pose()
            marker.pose.position.x = placement.center_x
            marker.pose.position.y = placement.center_y
            marker.pose.position.z = placement.center_z

            color_seed = (index * 53) % 255
            marker.color.r = (color_seed % 255) / 255.0
            marker.color.g = ((color_seed * 3) % 255) / 255.0
            marker.color.b = ((color_seed * 7) % 255) / 255.0
            marker.color.a = 0.85

            markers.append(marker)
        return markers

    def _create_pick_markers(self, pick_poses: List[Pose]) -> List[Marker]:
        markers: List[Marker] = []
        for index, pose in enumerate(pick_poses, start=1000):
            marker = Marker()
            marker.header.frame_id = self._frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "pick_points"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03

            marker.pose = pose
            marker.color.r = 1.0
            marker.color.g = 0.4
            marker.color.b = 0.1
            marker.color.a = 0.9

            markers.append(marker)
        return markers

    def _compute_pick_poses(self, source_spec: SourceSpec, boxes: List[Box]) -> List[Pose]:
        poses: List[Pose] = []
        for index, box in enumerate(boxes):
            row = index // source_spec.cols
            col = index % source_spec.cols

            pose = Pose()
            pose.position.x = source_spec.origin_x + col * source_spec.spacing_x
            pose.position.y = source_spec.origin_y + row * source_spec.spacing_y
            pose.position.z = source_spec.origin_z + box.size_z * 0.5
            poses.append(pose)
        return poses

    def _compute_place_poses(self, placements: List[Placement]) -> List[Pose]:
        poses: List[Pose] = []
        for placement in placements:
            pose = Pose()
            pose.position.x = placement.center_x
            pose.position.y = placement.center_y
            pose.position.z = placement.center_z
            poses.append(pose)
        return poses

    def _publish_pick_place_poses(self) -> None:
        pick_array = PoseArray()
        pick_array.header.frame_id = self._frame_id
        pick_array.header.stamp = self.get_clock().now().to_msg()
        pick_array.poses = self._pick_poses
        self._pick_pose_pub.publish(pick_array)

        place_array = PoseArray()
        place_array.header.frame_id = self._frame_id
        place_array.header.stamp = self.get_clock().now().to_msg()
        place_array.poses = self._place_poses
        self._place_pose_pub.publish(place_array)


def main() -> None:
    rclpy.init()
    node = PackingDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

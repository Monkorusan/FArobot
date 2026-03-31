from __future__ import annotations

import os
from typing import List, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from FArobot.packing import Bin, Box, Placement, plan_packing
from FArobot.packing_config import PackingConfig, SourceSpec, load_config




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

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self) -> None:
        super().__init__("robot_description_publisher")
        self.declare_parameter("robot_description", "")
        description = self.get_parameter("robot_description").get_parameter_value().string_value

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._pub = self.create_publisher(String, "robot_description", qos)

        msg = String()
        msg.data = description
        self._pub.publish(msg)
        self.get_logger().info("Published robot_description")


def main() -> None:
    rclpy.init()
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

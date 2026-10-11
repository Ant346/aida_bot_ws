"""Publish zero positions for the agrobot wheel joints.

Fixed links still come from robot_state_publisher; the wheels need a JointState.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")


class ZeroJointStates(Node):
    def __init__(self) -> None:
        super().__init__("zero_joint_states")
        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self._timer = self.create_timer(0.1, self._tick)

    def _tick(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [0.0] * len(JOINTS)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ZeroJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

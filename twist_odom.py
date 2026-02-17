#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """
    Convert quaternion (x,y,z,w) to yaw (rotation around Z) in radians.
    Assumes a standard ROS ENU frame and that roll/pitch are small or irrelevant (typical 2D robots).
    """
    # yaw (Z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class OdomObserver(Node):
    def __init__(self):
        super().__init__("odom_observer")
        # Depth 10 is fine for observation
        self.sub = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.get_logger().info("Listening to /odom (nav_msgs/msg/Odometry) ...")

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        yaw_deg = math.degrees(yaw)
        self.get_logger().info(
            f"x={p.x:.3f} m, y={p.y:.3f} m, yaw={yaw_deg:.1f} deg | "
            f"v_x={v.x:.3f} m/s, w_z={w.z:.3f} rad/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdomObserver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

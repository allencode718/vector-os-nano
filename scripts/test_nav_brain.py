#!/usr/bin/env python3
"""Test Vector OS Nano brain controlling Unity navigation stack."""
import sys
import time
import threading

import rclpy
from rclpy.node import Node

sys.path.insert(0, ".")
from vector_os_nano.core.nav_client import NavStackClient


def main():
    rclpy.init()
    node = rclpy.create_node("vector_os_nano_brain")

    nav = NavStackClient(node=node)
    print(f"NavStackClient available: {nav.is_available}")

    if not nav.is_available:
        print("ERROR: Nav stack not detected.")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Spin in background thread
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Wait for state estimation
    print("Waiting for state estimation...")
    for _ in range(20):
        time.sleep(0.5)
        odom = nav.get_state_estimation()
        if odom:
            print(f"Robot at: ({odom.x:.1f}, {odom.y:.1f})")
            break
    else:
        print("No state estimation received. Is nav stack running?")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Send goal
    x, y = 5.0, 0.0
    if len(sys.argv) >= 3:
        x, y = float(sys.argv[1]), float(sys.argv[2])

    print(f"\nSending goal: ({x}, {y})")
    print("Watching robot move (30s timeout)...")

    nav._goal_reached = False

    # Publish waypoint
    from geometry_msgs.msg import PointStamped
    msg = PointStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.point.x = float(x)
    msg.point.y = float(y)
    msg.point.z = 0.0
    nav._waypoint_pub.publish(msg)

    # Monitor position while waiting
    start = time.time()
    last_print = 0
    while time.time() - start < 30.0:
        time.sleep(0.5)
        odom = nav.get_state_estimation()
        if odom and time.time() - last_print > 2.0:
            dist = ((odom.x - x)**2 + (odom.y - y)**2)**0.5
            print(f"  pos=({odom.x:.1f}, {odom.y:.1f})  dist_to_goal={dist:.1f}m")
            last_print = time.time()
            if dist < 1.0:
                print("  CLOSE ENOUGH - goal reached!")
                break

        if nav._goal_reached:
            print("  /goal_reached received!")
            break

    odom = nav.get_state_estimation()
    if odom:
        print(f"\nFinal position: ({odom.x:.1f}, {odom.y:.1f})")

    print("Done.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

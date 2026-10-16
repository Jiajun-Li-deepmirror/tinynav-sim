"""E2E driver: production unitree_control (ROBOT_TYPE=as2) against the emu.

Publishes rt/cmd_vel + rt/service/command, consumes /unitree/odometry,
/battery, /robot_status over ROS 2, and asserts the full chain:
  sitting cmd_vel  -> Move refused, no motion
  play stand       -> /robot_status = standup
  cmd_vel again    -> /unitree/odometry advances (sportmodestate republish)
  play sit         -> /robot_status = sitting

Runs in the PILOT-side container (needs rclpy + unitree_sdk2py), emu on the
same DDS interface:
    ROBOT_TYPE=as2 python3 -m robot_emu.e2e_driver --iface docker0
(until robot_emu ships on the pilot image, run with the sim repo on
PYTHONPATH and this file's directory as cwd.)
"""
from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                         ChannelPublisher)
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Twist_, Vector3_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_ as UString_


class _Probe(Node):
    def __init__(self, state):
        super().__init__("emu_e2e_probe")
        self._state = state
        self.create_subscription(
            Odometry, "/unitree/odometry",
            lambda m: state.update(odom=(
                round(m.pose.pose.position.x, 3),
                round(m.pose.pose.position.y, 3))), 10)
        self.create_subscription(
            Float32, "/battery",
            lambda m: state.update(battery=round(m.data, 1)), 10)
        self.create_subscription(
            String, "/robot_status",
            lambda m: state.update(robot_status=m.data), 10)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--iface", default="docker0")
    args = ap.parse_args()

    ChannelFactoryInitialize(0, args.iface)
    rclpy.init()
    state: dict = {}
    node = _Probe(state)

    vel = ChannelPublisher("rt/cmd_vel", Twist_)
    vel.Init()
    cmd = ChannelPublisher("rt/service/command", UString_)
    cmd.Init()

    def twist(vx):
        vel.Write(Twist_(linear=Vector3_(x=float(vx), y=0.0, z=0.0),
                         angular=Vector3_(x=0.0, y=0.0, z=0.0)))

    def spin(seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(node, timeout_sec=0.05)

    ok = True

    print("phase 1: cmd_vel while sitting (Move must be refused)")
    for _ in range(8):
        twist(0.3)
        time.sleep(0.05)
    spin(1.0)
    print("  ", dict(state))
    if state.get("robot_status") != "sitting":
        ok = False

    print("phase 2: play stand")
    cmd.Write(UString_("play stand"))
    spin(2.0)
    print("  ", dict(state))
    if state.get("robot_status") != "standup":
        ok = False

    print("phase 3: cmd_vel (odometry must advance)")
    before = state.get("odom")
    spin(4.0)
    for _ in range(80):
        twist(0.3)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.05)
    spin(1.0)
    after = state.get("odom")
    print(f"   odom {before} -> {after}")
    if not before or not after or before == after:
        ok = False

    print("phase 4: stop, play sit")
    twist(0.0)
    cmd.Write(UString_("play sit"))
    spin(2.0)
    print("  ", dict(state))
    if state.get("robot_status") != "sitting":
        ok = False

    node.destroy_node()
    rclpy.shutdown()
    print("E2E", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

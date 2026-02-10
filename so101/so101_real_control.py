#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import math
import tempfile
from typing import Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

OUT_KEYS = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]

NAME2OUT = {
    "shoulder_pan":  "shoulder_pan.pos",
    "shoulder_lift": "shoulder_lift.pos",
    "elbow_flex":    "elbow_flex.pos",
    "wrist_flex":    "wrist_flex.pos",
    "wrist_roll":    "wrist_roll.pos",
    "gripper":       "gripper.pos",
}


 
def write_json_inplace(path: str, data: Dict[str, float]) -> None:
    d = os.path.dirname(os.path.abspath(path)) or "."
    payload = json.dumps(data, separators=(",", ":"), ensure_ascii=False)
 
    fd, tmppath = tempfile.mkstemp(dir=d, prefix=".cmd.", suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(payload)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmppath, path)  # atomic on same filesystem
    finally:
        try:
            os.remove(tmppath)
        except FileNotFoundError:
            pass


class JointStateToJson(Node):
    def __init__(self):
        super().__init__("so101_real_control_left")

        # ---- params ----
        self.declare_parameter("in_topic", "/so101/joint_states_left")
        self.declare_parameter("out_path", "/home/gkim451/ros2_ws/src/so101/gkim/demo/cmd.json")
        self.declare_parameter("write_hz", 30.0)
        self.declare_parameter("assume_deg", False)   # True: input is deg; False: input is rad -> convert to deg
        self.declare_parameter("missing_hold", True) # True: keep last value for missing keys

        self.in_topic = str(self.get_parameter("in_topic").value)
        self.out_path = str(self.get_parameter("out_path").value)
        self.write_hz = float(self.get_parameter("write_hz").value)
        self.assume_deg = bool(self.get_parameter("assume_deg").value)
        self.missing_hold = bool(self.get_parameter("missing_hold").value)

        # IMPORTANT: directory may be non-writable; do NOT os.makedirs() here.
        if not os.path.isfile(self.out_path):
            self.get_logger().error(
                f"out_path does not exist (must be a pre-created writable file): {self.out_path}"
            )

        # ---- state ----
        self.latest: Dict[str, float] = {k: 0.0 for k in OUT_KEYS}
        self.have_msg = False

        # ---- ROS I/O ----
        self.sub = self.create_subscription(JointState, self.in_topic, self._on_js, 10)

        dt = 1.0 / max(self.write_hz, 1e-6)
        self.timer = self.create_timer(dt, self._tick)

        unit = "deg" if self.assume_deg else "rad -> deg"
        self.get_logger().info(
            "SO101 joint_states -> cmd.json writer (in-place)\n"
            f"  Subscribing: {self.in_topic}\n"
            f"  Writing: {os.path.abspath(self.out_path)} @ {self.write_hz:.1f} Hz\n"
            f"  Input unit: {unit}\n"
            f"  missing_hold: {self.missing_hold}"
        )

    def _on_js(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            return

        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}

        updated: Dict[str, float] = {}
        for name, outk in NAME2OUT.items():
            if name in name_to_pos:
                val = float(name_to_pos[name])
                if not self.assume_deg:
                    val = val * 180.0 / math.pi
                updated[outk] = val

        if not updated:
            return

        if self.missing_hold:
            self.latest.update(updated)
        else:
            # if not holding, set missing ones to 0.0
            self.latest = {k: float(updated.get(k, 0.0)) for k in OUT_KEYS}

        self.have_msg = True

    def _tick(self):
        if not self.have_msg:
            return

        out = {k: float(self.latest.get(k, 0.0)) for k in OUT_KEYS}

        try:
            write_json_inplace(self.out_path, out)
        except Exception as e:
            self.get_logger().warn(f"Failed to write JSON (in-place): {e}")


def main():
    rclpy.init()
    node = JointStateToJson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
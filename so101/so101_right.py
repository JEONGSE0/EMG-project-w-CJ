#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, math, select, termios, tty, fcntl, atexit, signal, threading, queue, time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String, Int32
from sensor_msgs.msg import JointState

# ---------------- Names & utils ----------------
JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift",
    "elbow_flex", "wrist_flex",
    "wrist_roll", "gripper",
]
N = len(JOINT_NAMES)
d2r = math.radians

# Joint limits (rad)
LOW  = np.array([d2r(-110), d2r(-100), d2r(-96.83), d2r(-95),  d2r(-157.211), d2r(-10)],  np.float32)
HIGH = np.array([d2r(110),  d2r(100),  d2r(96.83),  d2r(95),   d2r(162.789),  d2r(100)],  np.float32)

def clamp(q):
    return np.minimum(np.maximum(q, LOW), HIGH)

DEFAULT_POSE = np.zeros(N, np.float32)

# ---------------- Safe terminal helper ----------------
class SafeTerminal:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.is_tty = os.isatty(self.fd)
        self.saved_attr = None
        self.saved_fl = None

    def __enter__(self):
        if not self.is_tty:
            return self
        self.saved_attr = termios.tcgetattr(self.fd)
        self.saved_fl = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        tty.setcbreak(self.fd)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.saved_fl | os.O_NONBLOCK)
        return self

    def __exit__(self, *_):
        self.restore()

    def restore(self):
        if not self.is_tty:
            return
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved_attr)
            fcntl.fcntl(self.fd, fcntl.F_SETFL, self.saved_fl)
        except Exception:
            pass

# ---------------- Nonblocking key reader ----------------
class KeyReader(threading.Thread):
    def __init__(self, out_q, poll_hz=300):
        super().__init__(daemon=True)
        self.q = out_q
        self.dt = 1.0 / max(1.0, float(poll_hz))
        self.stop_flag = threading.Event()
        self.st = SafeTerminal()

    def run(self):
        with self.st:
            if not self.st.is_tty:
                while not self.stop_flag.is_set():
                    r, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if r:
                        ch = sys.stdin.read(1)
                        self.q.put(ch)
                return

            while not self.stop_flag.is_set():
                r, _, _ = select.select([sys.stdin], [], [], self.dt)
                if r:
                    try:
                        ch = os.read(self.st.fd, 1).decode(errors="ignore")
                        if ch:
                            self.q.put(ch)
                    except BlockingIOError:
                        pass

    def stop(self):
        self.stop_flag.set()
        self.st.restore()

# ---------------- ROS node ----------------
class HandNode(Node):
    # gesture idx constants
    REST       = 0
    EXTENSION  = 1
    FLEXION    = 2
    ULNAR      = 3
    RADIAL     = 4
    FIST       = 5
    ABDUCTION  = 6
    ADDUCTION  = 7
    SUPINATION = 8
    PRONATION  = 9

    def __init__(self):
        super().__init__("so101_control_right")

        # ---- parameters ----
        self.declare_parameter("topic", "/so101/joint_command_right")
        self.declare_parameter("rate_hz", 100.0)

        # ---- move velocity ----
        self.declare_parameter("vmax", 0.08)

        self.declare_parameter("vmax_min", 0.01)
        self.declare_parameter("vmax_max", 2.0)
        self.declare_parameter("vmax_step", 0.02)

        self.declare_parameter("gesture_idx_topic", "/emg/gesture_idx")
        self.declare_parameter("gesture_topic", "/emg/gesture")
        self.declare_parameter("use_string_gesture", False)
        self.declare_parameter("debounce_sec", 0.02)

        self.declare_parameter("kb_hold_timeout_sec", 0.15)

        self.topic  = str(self.get_parameter("topic").value)
        rate_hz     = float(self.get_parameter("rate_hz").value)
        self.dt     = 1.0 / max(1e-6, rate_hz)

        self.vmax       = float(self.get_parameter("vmax").value)
        self.vmax_min   = float(self.get_parameter("vmax_min").value)
        self.vmax_max   = float(self.get_parameter("vmax_max").value)
        self.vmax_step  = float(self.get_parameter("vmax_step").value)

        self.gidx_topic = str(self.get_parameter("gesture_idx_topic").value)
        self.gstr_topic = str(self.get_parameter("gesture_topic").value)
        self.use_string = bool(self.get_parameter("use_string_gesture").value)
        self.debounce   = float(self.get_parameter("debounce_sec").value)

        self.kb_hold_timeout = float(self.get_parameter("kb_hold_timeout_sec").value)

        # ---- pubs/subs ----
        self.pub = self.create_publisher(JointState, self.topic, 10)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.sub_idx = self.create_subscription(Int32, self.gidx_topic, self._on_gesture_idx, qos)

        self.sub_str = None
        if self.use_string:
            self.sub_str = self.create_subscription(String, self.gstr_topic, self._on_gesture_str, qos)

        # ---- state ----
        self.msg = JointState()
        self.msg.name = JOINT_NAMES

        self.q_cmd = DEFAULT_POSE.copy()

        self._last_cmd_time = 0.0
        self._last_idx = self.REST
        self._last_str = None

        self.emg_enabled = True
        self._kb_active_until = 0.0

        # ✅ Gripper toggle state
        # start OPEN (100 deg)
        self._gripper_is_closed = False
        self._gripper_target_deg = 100.0

        # ✅ for edge-detect of FIST (rising edge)
        self._prev_idx_for_toggle = None

        # ---- keyboard ----
        self.keyq = queue.Queue()
        self.kr = KeyReader(self.keyq, poll_hz=300)
        self.kr.start()
        atexit.register(self.kr.stop)
        signal.signal(signal.SIGINT,  lambda *_: self._shutdown())
        signal.signal(signal.SIGTERM, lambda *_: self._shutdown())

        # ---- timer ----
        self.timer = self.create_timer(self.dt, self._tick)

        deg_per_sec = self.vmax * 180.0 / math.pi
        self.get_logger().info(
            "SO101 control\n"
            " Keys:\n"
            "   e : toggle EMG mode (ON=EMG only, OFF=Keyboard 0~9)\n"
            "   0~9 : gesture input (only when EMG mode OFF, hold-style with timeout)\n"
            "   r : reset all joints to 0 (always)\n"
            "   u/d : speed up/down (always)\n"
            "   x : quit\n"
            " Gripper: FIST toggles close/open (no longer forced by REST)\n"
            f" Subscribing gesture idx from: {self.gidx_topic}\n"
            f" EMG mode: {self.emg_enabled} | vmax(rad/s): {self.vmax:.3f} (~{deg_per_sec:.2f} deg/s)\n"
            f" Keyboard hold timeout: {self.kb_hold_timeout:.2f}s"
        )

    # ---------- helpers ----------
    def _shutdown(self):
        try:
            self.kr.stop()
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _set_all_zero(self):
        self.q_cmd = np.zeros(N, np.float32)
        self.q_cmd = clamp(self.q_cmd)
        self._last_idx = self.REST
        self._last_cmd_time = 0.0
        self._kb_active_until = 0.0

        # reset gripper toggle to OPEN
        self._gripper_is_closed = False
        self._gripper_target_deg = 100.0
        self._prev_idx_for_toggle = None

        self.get_logger().info("Reset: all joints -> 0 (gripper OPEN)")

    def _change_speed(self, direction: int):
        self.vmax = float(np.clip(self.vmax + direction * self.vmax_step, self.vmax_min, self.vmax_max))
        deg_per_sec = self.vmax * 180.0 / math.pi
        self.get_logger().info(f"vmax(rad/s) = {self.vmax:.3f} (~{deg_per_sec:.2f} deg/s)")

    def _apply_increment(self, joint_name: str, direction: float):
        i = JOINT_NAMES.index(joint_name)
        self.q_cmd[i] += float(direction) * self.vmax * self.dt

    def _apply_increment_pair(self, joint_a: str, joint_b: str, direction: float):
        ia = JOINT_NAMES.index(joint_a)
        ib = JOINT_NAMES.index(joint_b)
        step = float(direction) * self.vmax * self.dt
        self.q_cmd[ia] += step
        self.q_cmd[ib] += step

    def _gripper_to_target(self, target_deg: float):
        gi = JOINT_NAMES.index("gripper")
        target = d2r(target_deg)
        curr = float(self.q_cmd[gi])
        delta = target - curr
        step = np.clip(delta, -self.vmax * self.dt, self.vmax * self.dt)
        self.q_cmd[gi] = curr + step

    def _toggle_gripper_target(self):
        # closed target = -10 deg, open target = 100 deg
        self._gripper_is_closed = not self._gripper_is_closed
        self._gripper_target_deg = (-10.0 if self._gripper_is_closed else 100.0)
        self.get_logger().info(f"Gripper toggle -> {'CLOSE(-10deg)' if self._gripper_is_closed else 'OPEN(100deg)'}")

    def _set_gesture_idx_from_keyboard(self, idx: int):
        now = time.monotonic()
        if now - self._last_cmd_time < self.debounce:
            return
        self._last_cmd_time = now
        self._last_idx = int(idx)
        self._kb_active_until = now + self.kb_hold_timeout

    # ---------- inputs ----------
    def _handle_keys(self):
        while not self.keyq.empty():
            k = self.keyq.get_nowait()

            if k == 'x':
                self._shutdown()

            elif k == 'r':
                self._set_all_zero()

            elif k == 'u':
                self._change_speed(+1)

            elif k == 'd':
                self._change_speed(-1)

            elif k == 'e':
                self.emg_enabled = not self.emg_enabled
                self._last_idx = self.REST
                self._kb_active_until = 0.0
                self._prev_idx_for_toggle = None
                self.get_logger().info(f"EMG mode = {self.emg_enabled} (ON=EMG only, OFF=Keyboard 0~9)")

            elif ('0' <= k <= '9'):
                if not self.emg_enabled:
                    self._set_gesture_idx_from_keyboard(int(k))

    def _on_gesture_idx(self, msg: Int32):
        if not self.emg_enabled:
            return

        now = time.monotonic()
        if now - self._last_cmd_time < self.debounce:
            return

        self._last_cmd_time = now
        self._last_idx = int(msg.data)

    def _on_gesture_str(self, msg: String):
        self._last_str = msg.data

    # ---------- keyboard hold -> REST ----------
    def _apply_keyboard_hold_fallback(self):
        if self.emg_enabled:
            return
        now = time.monotonic()
        if now > self._kb_active_until:
            self._last_idx = self.REST

    # ---------- gesture -> joint control ----------
    def _apply_gesture_control(self):
        idx = self._last_idx
        if idx is None:
            return

        # Gripper toggle on FIST rising edge
        if idx == self.FIST and self._prev_idx_for_toggle != self.FIST:
            self._toggle_gripper_target()

        # Always drive gripper toward the current target (holds state even when REST keeps coming)
        self._gripper_to_target(self._gripper_target_deg)

        # update prev idx for edge detection
        self._prev_idx_for_toggle = idx

        # Rest -> hold other joints (gripper 제외)
        if idx == self.REST:
            return


        # shoulder_pan: Ulnar +, Radial -
        if idx == self.ULNAR:
            self._apply_increment("shoulder_lift", +1.0)
            self._apply_increment("elbow_flex",   -1.0)
        elif idx == self.RADIAL:
            self._apply_increment("shoulder_lift", -1.0)
            self._apply_increment("elbow_flex",    +1.0)

        # shoulder_lift + elbow_flex opposite: Flexion(+/-), Extension(-/+)
        if idx == self.FLEXION:
            self._apply_increment("shoulder_pan", -1.0)
        elif idx == self.EXTENSION:
            self._apply_increment("shoulder_pan", +1.0)

        # wrist_flex: Adduction +, Abduction -
        if idx == self.ADDUCTION:
            self._apply_increment("wrist_flex", +1.0)
        elif idx == self.ABDUCTION:
            self._apply_increment("wrist_flex", -1.0)

        # wrist_roll: Pronation +, Supination -
        if idx == self.PRONATION:
            self._apply_increment("wrist_roll", +1.0)
        elif idx == self.SUPINATION:
            self._apply_increment("wrist_roll", -1.0)


    # ---------- main loop ----------
    def _tick(self):
        self._handle_keys()
        self._apply_keyboard_hold_fallback()
        self._apply_gesture_control()

        self.q_cmd = clamp(self.q_cmd)
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.position = self.q_cmd.tolist()
        self.msg.velocity = []
        self.msg.effort = []
        self.pub.publish(self.msg)

def main():
    rclpy.init()
    node = HandNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.kr.stop()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass

if __name__ == "__main__":
    main()
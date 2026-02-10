#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EMG real-time classifier node

Input:
  /emg/signal (std_msgs/Float32MultiArray) : single sample, len>=8 (use first 8)

Pipeline:
  (optional) RealtimeFilter.process(x[N,8])
  -> RingBuffer.push(x[N,8], t[N])
  -> when buffer ready:
       every pred_hop samples:
         window = rb.get_window()  # (WIN_SAMPLES, 8)
         feats  = compute_features(window, fs)
         proba/pred from sklearn Pipeline (joblib)
         DecisionLayer.step(proba)  # confirm_n=3, hold_hops=8 (0.8s hold at 10Hz)
         publish

Warmup:
  from first valid sample time, block inference/publish for warmup_sec seconds (buffer & filter fill only)

Outputs:
  /emg/gesture             (String)  "Label (0.95)"
  /emg/gesture_idx         (Int32)   0..9 (final command)
  /emg/gesture_confidence  (Float32) 0..1 (proba of final command if available)
  /emg/gesture_probs       (Float32MultiArray) length 10 if publish_probs=True

Model:
  default: <pkg_share>/models/re_model_2.joblib
  joblib file is a dict with keys including "model" (Pipeline) and metadata
"""

from __future__ import annotations

from typing import List, Optional, Tuple, Any, Dict
import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from std_msgs.msg import Float32MultiArray, String, Int32, Float32
from ament_index_python.packages import get_package_share_directory

# ---------------------------
# joblib (required)
# ---------------------------
try:
    import joblib  # type: ignore
except Exception as e:
    raise RuntimeError(f"joblib import failed: {e!r}")

# ---------------------------
# package imports (match your files)
# ---------------------------
try:
    from emg_device.config import (
        FS as CFG_FS,
        WIN_SAMPLES as CFG_WIN,
        HOP_SAMPLES as CFG_HOP,
        N_CH as CFG_NCH,
        N_CLASSES as CFG_NCLS,
        CONFIRM_N as CFG_CONFIRM_N,
        HOLD_HOPS as CFG_HOLD_HOPS,
    )
    from emg_device.ring_buffer import RingBuffer
    from emg_device.features import compute_features
    from emg_device.decision import DecisionLayer, DecisionState
except Exception:
    from .config import (  # type: ignore
        FS as CFG_FS,
        WIN_SAMPLES as CFG_WIN,
        HOP_SAMPLES as CFG_HOP,
        N_CH as CFG_NCH,
        N_CLASSES as CFG_NCLS,
        CONFIRM_N as CFG_CONFIRM_N,
        HOLD_HOPS as CFG_HOLD_HOPS,
    )
    from .ring_buffer import RingBuffer  # type: ignore
    from .features import compute_features  # type: ignore
    from .decision import DecisionLayer, DecisionState  # type: ignore

# ---------------------------
# optional realtime filter
# ---------------------------
_HAS_FILTER = False
_FILTER_IMPORT_ERR: Optional[Exception] = None
try:
    from emg_device.real_time_filter import RealtimeFilter  # type: ignore
    _HAS_FILTER = True
except Exception as e1:
    try:
        from .real_time_filter import RealtimeFilter  # type: ignore
        _HAS_FILTER = True
    except Exception as e2:
        _FILTER_IMPORT_ERR = e2


def _ros_time_sec(node: Node) -> float:
    return float(node.get_clock().now().nanoseconds) * 1e-9


def _unwrap_joblib_model(obj: Any) -> Tuple[Any, Dict[str, Any]]:
    """
    Your joblib file loads as dict:
      keys: ['model', 'fs', 'win_sec', 'hop_sec', 'win_samples', 'hop_samples', ...]
    Return (model, meta_dict).
    """
    if isinstance(obj, dict):
        if "model" not in obj:
            raise RuntimeError(f"joblib object is dict but missing key 'model'. keys={list(obj.keys())}")
        model = obj["model"]
        meta = obj
        return model, meta
    # Sometimes joblib directly stores estimator/pipeline
    return obj, {}


def _ensure_len10(probs: np.ndarray) -> np.ndarray:
    probs = np.asarray(probs, dtype=np.float32).reshape(-1)
    out = np.zeros((10,), dtype=np.float32)
    out[: min(10, probs.shape[0])] = probs[: min(10, probs.shape[0])]
    return out


class EmgClassifierNode(Node):
    def __init__(self) -> None:
        super().__init__("emg_classifier_node")

        # ---------------- Parameters ----------------
        self.declare_parameter(
            "class_names",
            [
                "Rest",
                "Extension",
                "Flexion",
                "Ulnar Deviation",
                "Radial Deviation",
                "Fist",
                "Abduction",
                "Adduction",
                "Supination",
                "Pronation",
            ],
        )

        self.declare_parameter("input_topic", "/emg/signal")
        self.declare_parameter("num_channels", int(CFG_NCH))  # expected 8

        # window / hop (samples)
        self.declare_parameter("fs", int(CFG_FS))
        self.declare_parameter("window_size", int(CFG_WIN))   # usually 500
        self.declare_parameter("pred_hop", int(CFG_HOP))      # usually 50 (10Hz @ 500Hz)

        # warmup (sec)
        self.declare_parameter("warmup_sec", 10.0)

        # filter
        self.declare_parameter("enable_filter", True)

        # model path
        self.declare_parameter("model_path", "")              # if empty -> pkg_share/models/<model_filename>
        self.declare_parameter("model_filename", "quick_modeljoblib")

        # decision parameters (3 consecutive then hold 0.8s = 8 hops at 10Hz)
        self.declare_parameter("confirm_n", int(CFG_CONFIRM_N))
        self.declare_parameter("hold_hops", int(CFG_HOLD_HOPS))

        # outputs
        self.declare_parameter("gesture_topic", "/emg/gesture")
        self.declare_parameter("gesture_idx_topic", "/emg/gesture_idx")
        self.declare_parameter("gesture_conf_topic", "/emg/gesture_confidence")
        self.declare_parameter("publish_probs", False)
        self.declare_parameter("gesture_probs_topic", "/emg/gesture_probs")

        # misc
        self.declare_parameter("qos_depth", 50)
        self.declare_parameter("debug", 1)  # 0=min, 1=event, 2=1Hz summary

        # ---------------- Read params ----------------
        self.class_names: List[str] = list(
            self.get_parameter("class_names").get_parameter_value().string_array_value
        )

        self.input_topic: str = str(self.get_parameter("input_topic").value)
        self.num_channels: int = int(self.get_parameter("num_channels").value)

        self.fs: int = int(self.get_parameter("fs").value)
        self.window_size: int = int(self.get_parameter("window_size").value)
        self.pred_hop: int = int(self.get_parameter("pred_hop").value)

        self.warmup_sec: float = float(self.get_parameter("warmup_sec").value)

        self.enable_filter: bool = bool(self.get_parameter("enable_filter").value)

        model_path_param: str = str(self.get_parameter("model_path").value)
        model_filename: str = str(self.get_parameter("model_filename").value)

        confirm_n: int = int(self.get_parameter("confirm_n").value)
        hold_hops: int = int(self.get_parameter("hold_hops").value)

        self.gesture_topic: str = str(self.get_parameter("gesture_topic").value)
        self.gesture_idx_topic: str = str(self.get_parameter("gesture_idx_topic").value)
        self.gesture_conf_topic: str = str(self.get_parameter("gesture_conf_topic").value)
        self.publish_probs: bool = bool(self.get_parameter("publish_probs").value)
        self.gesture_probs_topic: str = str(self.get_parameter("gesture_probs_topic").value)

        qos_depth: int = int(self.get_parameter("qos_depth").value)
        self.debug: int = int(self.get_parameter("debug").value)

        # ---------------- QoS ----------------
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # ---------------- Runtime counters/state ----------------
        self._recv_count = 0
        self._infer_count = 0
        self._pub_count = 0
        self._err_count = 0
        self._last_cb_wall: Optional[float] = None

        self._first_sample_time: Optional[float] = None
        self._infer_enabled = False
        self._hop_counter = 0

        # ---------------- Filter ----------------
        self._filter: Optional[Any] = None
        if self.enable_filter:
            if not _HAS_FILTER:
                self.get_logger().warn(f"[init] enable_filter=True but RealtimeFilter import failed: {_FILTER_IMPORT_ERR!r}")
                self.get_logger().warn("[init] -> filter OFF (RAW).")
                self.enable_filter = False
            else:
                try:
                    self._filter = RealtimeFilter(fs=float(self.fs))
                    self.get_logger().info(f"[init] RealtimeFilter enabled (fs={self.fs}).")
                except Exception as e:
                    self.get_logger().warn(f"[init] Filter init failed: {e!r} -> filter OFF (RAW).")
                    self.enable_filter = False
                    self._filter = None

        # ---------------- RingBuffer / Decision ----------------
        # IMPORTANT: Your RingBuffer uses config.WIN_SAMPLES / config.N_CH internally.
        # So we warn if ros params mismatch config to avoid hidden bugs.
        if self.window_size != int(CFG_WIN):
            self.get_logger().warn(
                f"[init] window_size(param={self.window_size}) != config.WIN_SAMPLES({int(CFG_WIN)}). "
                f"RingBuffer will still use config.WIN_SAMPLES. (추천: config.py 값과 맞추기)"
            )
        if self.num_channels != int(CFG_NCH):
            self.get_logger().warn(
                f"[init] num_channels(param={self.num_channels}) != config.N_CH({int(CFG_NCH)}). "
                f"RingBuffer will still use config.N_CH."
            )

        self._rb = RingBuffer()
        self._decision = DecisionLayer(confirm_n=confirm_n, hold_hops=hold_hops)

        # ---------------- Model load ----------------
        if model_path_param:
            model_path = model_path_param
        else:
            pkg_share = get_package_share_directory("emg_device")
            model_path = os.path.join(pkg_share, "models", model_filename)

        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f"model_path not found: {model_path}\n"
                f"(hint) if running from source, pass -p model_path:=/home/meow/ros2_ws/src/emg_device/emg_device/models/quick_modeljoblib "
                f"or install package with model included in share/"
            )

        raw_obj = joblib.load(model_path)
        self._model, self._model_meta = _unwrap_joblib_model(raw_obj)

        # If the stored metadata disagrees, log it (but don't hard-fail).
        try:
            meta_fs = float(self._model_meta.get("fs", self.fs)) if self._model_meta else self.fs
            meta_win = int(self._model_meta.get("win_samples", self.window_size)) if self._model_meta else self.window_size
            meta_hop = int(self._model_meta.get("hop_samples", self.pred_hop)) if self._model_meta else self.pred_hop
            if abs(meta_fs - float(self.fs)) > 1e-6:
                self.get_logger().warn(f"[init] model meta fs={meta_fs} != param fs={self.fs}")
            if meta_win != self.window_size:
                self.get_logger().warn(f"[init] model meta win_samples={meta_win} != param window_size={self.window_size}")
            if meta_hop != self.pred_hop:
                self.get_logger().warn(f"[init] model meta hop_samples={meta_hop} != param pred_hop={self.pred_hop}")
        except Exception:
            pass

        self._has_proba = hasattr(self._model, "predict_proba")
        self._classes_: Optional[np.ndarray] = None
        if hasattr(self._model, "classes_"):
            try:
                self._classes_ = np.asarray(getattr(self._model, "classes_"), dtype=int)
            except Exception:
                self._classes_ = None

        if not self._has_proba:
            self.get_logger().warn("[init] model has no predict_proba(). Will publish one-hot probs/conf=1.0.")

        self.get_logger().info(
            f"[init] model loaded: {model_path} | type={type(self._model).__name__} | has_proba={self._has_proba} | classes_={self._classes_}"
        )

        # ---------------- ROS IO ----------------
        self.sub = self.create_subscription(Float32MultiArray, self.input_topic, self._on_emg, qos)
        self.pub_gesture = self.create_publisher(String, self.gesture_topic, qos)
        self.pub_gesture_idx = self.create_publisher(Int32, self.gesture_idx_topic, qos)
        self.pub_gesture_conf = self.create_publisher(Float32, self.gesture_conf_topic, qos)
        self.pub_gesture_probs = (
            self.create_publisher(Float32MultiArray, self.gesture_probs_topic, qos)
            if self.publish_probs else None
        )

        if self.debug >= 2:
            self.create_timer(1.0, self._summary_tick)

        self.get_logger().info(
            "[init] params: "
            f"input={self.input_topic}, fs={self.fs}, window_size={self.window_size}, pred_hop={self.pred_hop}, "
            f"filter={self.enable_filter}, warmup_sec={self.warmup_sec}, debug={self.debug}, publish_probs={self.publish_probs}"
        )

    # ---------------- Diagnostics ----------------
    def _summary_tick(self) -> None:
        now = time.time()
        age = "no data yet" if self._last_cb_wall is None else f"{now - self._last_cb_wall:.2f}s ago"
        self.get_logger().info(
            f"[diag] recv={self._recv_count} infer={self._infer_count} pub={self._pub_count} err={self._err_count} "
            f"rb_ready={bool(self._rb.ready)} infer_enabled={self._infer_enabled} last_cb={age}"
        )

    # ---------------- Callback ----------------
    def _on_emg(self, msg: Float32MultiArray) -> None:
        self._recv_count += 1
        self._last_cb_wall = time.time()

        x = np.asarray(msg.data, dtype=np.float64)
        if x.size < 8:
            if self.debug >= 1:
                self.get_logger().warn(f"[cb] invalid length={x.size}, expected>=8")
            return
        x = x[:8]

        if not np.isfinite(x).all():
            if self.debug >= 1:
                self.get_logger().warn("[cb] non-finite detected (NaN/Inf) -> dropped")
            return

        # warmup start (first valid sample)
        if self._first_sample_time is None:
            self._first_sample_time = _ros_time_sec(self)
            self._infer_enabled = False
            self._hop_counter = 0
            try:
                self._rb.reset()
            except Exception:
                # fallback (should not happen with your RingBuffer)
                self._rb = RingBuffer()
            if self._filter is not None and hasattr(self._filter, "reset"):
                try:
                    self._filter.reset()
                except Exception:
                    pass
            self._decision.reset()
            if self.debug >= 1:
                self.get_logger().info(f"[warmup] started (target {self.warmup_sec:.1f}s)")

        # optional filter: MUST pass shape (N, ch) not (ch,)
        if self.enable_filter and self._filter is not None:
            try:
                xin = x.reshape(1, 8)  # (1,8)
                y = self._filter.process(xin)
                y = np.asarray(y, dtype=np.float64)
                x = y.reshape(-1)[:8]  # back to (8,)
            except Exception as e:
                self._err_count += 1
                if self.debug >= 1:
                    self.get_logger().error(f"[filter] exception: {e}")
                return

        # push into ring buffer (expects (N, ch), (N,))
        try:
            t = np.asarray([_ros_time_sec(self)], dtype=np.float64)
            X = x.reshape(1, 8).astype(np.float64)
            self._rb.push(X, t)
        except Exception as e:
            self._err_count += 1
            if self.debug >= 1:
                self.get_logger().error(f"[ringbuffer] exception: {e}")
            return

        # buffer ready? (property, not callable)
        if not bool(self._rb.ready):
            if self.debug >= 1 and (self._recv_count % 50 == 0):
                self.get_logger().info("[cb] filling window...")
            return

        # warmup gate
        if not self._infer_enabled:
            assert self._first_sample_time is not None
            remain = self.warmup_sec - (_ros_time_sec(self) - self._first_sample_time)
            if remain > 0.0:
                if self.debug >= 1 and (abs(remain - round(remain)) < 0.05 or remain < 1.0):
                    self.get_logger().info(f"[warmup] waiting {remain:.1f}s")
                return
            self._infer_enabled = True
            if self.debug >= 1:
                self.get_logger().info("[warmup] done; inference enabled")

        # hop control (sample-count based)
        self._hop_counter += 1
        if self._hop_counter < self.pred_hop:
            return
        self._hop_counter = 0

        # ---------- Inference ----------
        try:
            win_x, _win_t = self._rb.get_window()  # (WIN_SAMPLES, 8)
            if win_x is None:
                return

            feats = compute_features(win_x, fs=float(self.fs))  # (88,)
            Xfeat = np.asarray(feats, dtype=np.float64).reshape(1, -1)

            if self._has_proba:
                proba_raw = np.asarray(self._model.predict_proba(Xfeat)[0], dtype=np.float32).reshape(-1)

                # If model has classes_, map to 0..9 order
                probs = np.zeros((10,), dtype=np.float32)
                if self._classes_ is not None and proba_raw.shape[0] == self._classes_.shape[0]:
                    for c, p in zip(self._classes_, proba_raw):
                        ci = int(c)
                        if 0 <= ci < 10:
                            probs[ci] = float(p)
                else:
                    probs = _ensure_len10(proba_raw)

            else:
                pred = int(self._model.predict(Xfeat)[0])
                probs = np.zeros((10,), dtype=np.float32)
                if 0 <= pred < 10:
                    probs[pred] = 1.0

            # decision layer: 3-confirm then hold 0.8s(=8 hops)
            st: DecisionState = self._decision.step(probs)
            final_idx = int(st.final_cmd)

            # confidence: probability of final command (during hold, still report that class prob)
            conf = float(probs[final_idx]) if 0 <= final_idx < 10 else float(st.raw_conf)

            label = self.class_names[final_idx] if 0 <= final_idx < len(self.class_names) else str(final_idx)

            self._infer_count += 1

        except Exception as e:
            self._err_count += 1
            if self.debug >= 1:
                self.get_logger().error(f"[infer] exception: {e}")
            return

        # ---------- Publish ----------
        try:
            out_str = String()
            out_str.data = f"{label} ({conf:.2f})"
            self.pub_gesture.publish(out_str)

            out_idx = Int32()
            out_idx.data = final_idx
            self.pub_gesture_idx.publish(out_idx)

            out_conf = Float32()
            out_conf.data = conf
            self.pub_gesture_conf.publish(out_conf)

            if self.pub_gesture_probs is not None:
                arr = Float32MultiArray()
                arr.data = probs.astype(np.float32).tolist()  # length 10
                self.pub_gesture_probs.publish(arr)

            self._pub_count += 1
            if self.debug >= 1:
                # confirm/hold debug info is useful for your “3연속 + 0.8s 유지” 확인
                self.get_logger().info(
                    f"[pub] final={label} conf={conf*100:.1f}% "
                    f"(raw={st.raw_pred}:{st.raw_conf*100:.1f}%, confirm={st.confirm_count}, hold_rem={st.hold_remaining_hops})"
                )

        except Exception as e:
            self._err_count += 1
            if self.debug >= 1:
                self.get_logger().error(f"[publish] exception: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EmgClassifierNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        # Jazzy에서 "already shutdown" 튀는 경우가 있어서 예외 삼킴
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
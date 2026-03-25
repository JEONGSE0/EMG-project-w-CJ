import os
import threading
from collections import deque

import numpy as np
import pyqtgraph as pg
import rclpy

from PySide6.QtCore import QFile, QTimer
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QVBoxLayout
from ament_index_python.packages import get_package_share_directory

from .process_manager import ProcessManager
from .config import COMMANDS
from .ros_node import RosSignals, GuiRosNode


class MainWindow:
    def __init__(self):
        self.pm = ProcessManager()
        self.ui = self.load_ui()

        self.right_warmup_active = False
        self.left_warmup_active = False

        self.init_labels()
        self.init_graphs()
        self.connect_signals()
        self.init_ros()

        self.timer = QTimer()
        self.timer.timeout.connect(self.poll_logs)
        self.timer.start(100)

    def load_ui(self):
        package_share = get_package_share_directory("emg_system_gui")
        ui_path = os.path.join(package_share, "ui", "main_window.ui")

        loader = QUiLoader()
        ui_file = QFile(ui_path)
        if not ui_file.open(QFile.ReadOnly):
            raise RuntimeError(f"Failed to open UI file: {ui_path}")

        ui = loader.load(ui_file)
        ui_file.close()

        if ui is None:
            raise RuntimeError(f"Failed to load UI file: {ui_path}")

        return ui

    def init_labels(self):
        self.ui.label_right_status.setText("disconnected 💤")
        self.ui.label_left_status.setText("disconnected 💤")
        self.ui.label_right_gesture.setText("-")
        self.ui.label_left_gesture.setText("-")

        if hasattr(self.ui, "label_speed_right"):
            self.ui.label_speed_right.setText("1.0")
        if hasattr(self.ui, "label_speed_left"):
            self.ui.label_speed_left.setText("1.0")

    def init_graphs(self):
        pg.setConfigOptions(antialias=False)

        pg.setConfigOption('background', 'w')   
        pg.setConfigOption('foreground', 'k')   

        self.graph_buffer_len = 300

        self.right_signal_buffers = [
            deque([0.0] * self.graph_buffer_len, maxlen=self.graph_buffer_len)
            for _ in range(8)
        ]
        self.left_signal_buffers = [
            deque([0.0] * self.graph_buffer_len, maxlen=self.graph_buffer_len)
            for _ in range(8)
        ]

        self.right_curves = []
        self.left_curves = []
        self.right_plots = []
        self.left_plots = []

        for i in range(8):
            right_widget = getattr(self.ui, f"widget_right_ch{i+1}")
            left_widget = getattr(self.ui, f"widget_left_ch{i+1}")

            right_plot = pg.PlotWidget()
            left_plot = pg.PlotWidget()

            self._setup_single_plot(right_plot, "")
            self._setup_single_plot(left_plot, "")

            # 기존 레이아웃이 있으면 제거 없이 새 레이아웃 생성 시 경고 날 수 있어서 체크
            if right_widget.layout() is None:
                right_layout = QVBoxLayout(right_widget)
                right_layout.setContentsMargins(0, 0, 0, 0)
            else:
                right_layout = right_widget.layout()

            if left_widget.layout() is None:
                left_layout = QVBoxLayout(left_widget)
                left_layout.setContentsMargins(0, 0, 0, 0)
            else:
                left_layout = left_widget.layout()

            right_layout.addWidget(right_plot)
            left_layout.addWidget(left_plot)

            right_curve = right_plot.plot(
                np.array(self.right_signal_buffers[i]),
                pen=pg.mkPen(color=(0, 120, 255), width=2)
            )
            left_curve = left_plot.plot(
                np.array(self.left_signal_buffers[i]),
                pen=pg.mkPen(color=(0, 120, 255), width=2)
            )

            self.right_plots.append(right_plot)
            self.left_plots.append(left_plot)
            self.right_curves.append(right_curve)
            self.left_curves.append(left_curve)

    def _setup_single_plot(self, plot_widget, title: str):
        plot_widget.showGrid(x=False, y=True, alpha=0.2)
        plot_widget.setMenuEnabled(False)
        plot_widget.setMouseEnabled(x=False, y=False)
        plot_widget.hideButtons()
        plot_widget.enableAutoRange(axis='y', enable=False)
        plot_widget.getPlotItem().setContentsMargins(0, 0, 0, 0)
        plot_widget.getPlotItem().hideAxis('bottom')

        axis = plot_widget.getPlotItem()

        axis.getAxis('left').setStyle(tickFont=pg.QtGui.QFont("Arial", 6))

    def _normalize_status(self, status: str) -> str:
        s = status.strip().lower()

        if "disconnect" in s:
            return "disconnected"
        if "connect" in s:
            return "connected"
        if "scan" in s:
            return "scanning"
        if "error" in s or "fail" in s:
            return "error"
        if "idle" in s:
            return "idle"

        return s

    def init_ros(self):
        self.ros_signals = RosSignals()

        self.ros_signals.right_ble_status.connect(self.update_right_ble_status)
        self.ros_signals.left_ble_status.connect(self.update_left_ble_status)

        self.ros_signals.right_warmup_remaining.connect(self.update_right_warmup)
        self.ros_signals.left_warmup_remaining.connect(self.update_left_warmup)

        self.ros_signals.right_gesture.connect(self.update_right_gesture)
        self.ros_signals.left_gesture.connect(self.update_left_gesture)

        self.ros_signals.right_filtered_signal.connect(self.update_right_filtered_signal)
        self.ros_signals.left_filtered_signal.connect(self.update_left_filtered_signal)

        self.ros_signals.right_current_speed.connect(self.update_right_speed_label)
        self.ros_signals.left_current_speed.connect(self.update_left_speed_label)

        if not rclpy.ok():
            rclpy.init(args=None)

        self.ros_node = GuiRosNode(self.ros_signals)
        self.ros_thread = threading.Thread(
            target=rclpy.spin,
            args=(self.ros_node,),
            daemon=True,
        )
        self.ros_thread.start()

    def connect_signals(self):
        # EMG connect/disconnect
        self.ui.btn_connect_right_emg.clicked.connect(
            lambda: self.run_process("right_emg", COMMANDS["connect_right_emg"])
        )
        self.ui.btn_disconnect_right_emg.clicked.connect(
            lambda: self.stop_process("right_emg")
        )

        self.ui.btn_connect_left_emg.clicked.connect(
            lambda: self.run_process("left_emg", COMMANDS["connect_left_emg"])
        )
        self.ui.btn_disconnect_left_emg.clicked.connect(
            lambda: self.stop_process("left_emg")
        )

        # Shadow hand
        if hasattr(self.ui, "btn_shadow_right_on"):
            self.ui.btn_shadow_right_on.clicked.connect(
                lambda: self.run_process("shadow_right", COMMANDS["shadow_right"])
            )
        if hasattr(self.ui, "btn_shadow_right_off"):
            self.ui.btn_shadow_right_off.clicked.connect(
                lambda: self.stop_process("shadow_right")
            )

        if hasattr(self.ui, "btn_shadow_left_on"):
            self.ui.btn_shadow_left_on.clicked.connect(
                lambda: self.run_process("shadow_left", COMMANDS["shadow_left"])
            )
        if hasattr(self.ui, "btn_shadow_left_off"):
            self.ui.btn_shadow_left_off.clicked.connect(
                lambda: self.stop_process("shadow_left")
            )

        # SO101 sim
        if hasattr(self.ui, "btn_so101_sim_right_on"):
            self.ui.btn_so101_sim_right_on.clicked.connect(
                lambda: self.run_process("so101_right", COMMANDS["so101_right"])
            )
        if hasattr(self.ui, "btn_so101_sim_right_off"):
            self.ui.btn_so101_sim_right_off.clicked.connect(
                lambda: self.stop_process("so101_right")
            )

        if hasattr(self.ui, "btn_so101_sim_left_on"):
            self.ui.btn_so101_sim_left_on.clicked.connect(
                lambda: self.run_process("so101_left", COMMANDS["so101_left"])
            )
        if hasattr(self.ui, "btn_so101_sim_left_off"):
            self.ui.btn_so101_sim_left_off.clicked.connect(
                lambda: self.stop_process("so101_left")
            )

        # SO101 real
        if hasattr(self.ui, "btn_so101_real_right_on"):
            self.ui.btn_so101_real_right_on.clicked.connect(
                lambda: self.run_process("so101_real_follow_right", COMMANDS["so101_real_follow_right"])
            )
        if hasattr(self.ui, "btn_so101_real_right_off"):
            self.ui.btn_so101_real_right_off.clicked.connect(
                lambda: self.stop_process("so101_real_follow_right")
            )

        if hasattr(self.ui, "btn_so101_real_left_on"):
            self.ui.btn_so101_real_left_on.clicked.connect(
                lambda: self.run_process("so101_real_follow_left", COMMANDS["so101_real_follow_left"])
            )
        if hasattr(self.ui, "btn_so101_real_left_off"):
            self.ui.btn_so101_real_left_off.clicked.connect(
                lambda: self.stop_process("so101_real_follow_left")
            )

        # placeholder control buttons
        if hasattr(self.ui, "btn_speed_up_right"):
            self.ui.btn_speed_up_right.clicked.connect(
                lambda: self.change_right_speed(+0.02)
            )
        if hasattr(self.ui, "btn_speed_down_right"):
            self.ui.btn_speed_down_right.clicked.connect(
                lambda: self.change_right_speed(-0.02)
            )
        if hasattr(self.ui, "btn_reset_right"):
            self.ui.btn_reset_right.clicked.connect(
                self.reset_right_pose
            )

        if hasattr(self.ui, "btn_speed_up_left"):
            self.ui.btn_speed_up_left.clicked.connect(
                lambda: self.change_left_speed(+0.02)
            )
        if hasattr(self.ui, "btn_speed_down_left"):
            self.ui.btn_speed_down_left.clicked.connect(
                lambda: self.change_left_speed(-0.02)
            )
        if hasattr(self.ui, "btn_reset_left"):
            self.ui.btn_reset_left.clicked.connect(
                self.reset_left_pose
            )

    def change_right_speed(self, delta: float):
        self.ros_node.send_right_speed_delta(delta)
        self.append_log(f"[SO101 RIGHT] speed delta {delta:+.2f}")

    def change_left_speed(self, delta: float):
        self.ros_node.send_left_speed_delta(delta)
        self.append_log(f"[SO101 LEFT] speed delta {delta:+.2f}")

    def reset_right_pose(self):
        self.ros_node.send_right_reset()
        self.append_log("[SO101 RIGHT] reset requested")

    def reset_left_pose(self):
        self.ros_node.send_left_reset()
        self.append_log("[SO101 LEFT] reset requested")

    def update_right_speed_label(self, speed: float):
        if hasattr(self.ui, "label_speed_right"):
            self.ui.label_speed_right.setText(f"{speed:.2f}")

    def update_left_speed_label(self, speed: float):
        if hasattr(self.ui, "label_speed_left"):
            self.ui.label_speed_left.setText(f"{speed:.2f}")
            
    def _update_curve_with_autoscale(self, curve, buffer_data, plot_widget):
        arr = np.array(buffer_data, dtype=np.float32)
        curve.setData(arr)

        if arr.size == 0:
            return

        low = float(np.percentile(arr, 5))
        high = float(np.percentile(arr, 95))

        if abs(high - low) < 1e-6:
            center = high
            plot_widget.setYRange(center - 1.0, center + 1.0, padding=0.0)
            return

        margin = 0.2 * (high - low)
        plot_widget.setYRange(low - margin, high + margin, padding=0.0)

    

    def show(self):
        self.ui.show()

    def append_log(self, text: str):
        if hasattr(self.ui, "text_log"):
            self.ui.text_log.append(text)
        else:
            print(text)

    def run_process(self, name, command):
        msg = self.pm.start(name, command)
        self.append_log(msg)

        if name == "right_emg":
            self.ui.label_right_status.setText("Scanning...🛰️")
            self.ui.label_right_gesture.setText("-")
            self.right_warmup_active = False
            self.clear_right_graphs()

        elif name == "left_emg":
            self.ui.label_left_status.setText("Scanning...🛰️")
            self.ui.label_left_gesture.setText("-")
            self.left_warmup_active = False
            self.clear_left_graphs()

    def stop_process(self, name):
        msg = self.pm.stop(name)
        self.append_log(msg)

        if name == "right_emg":
            self.ui.label_right_status.setText("disconnected 💤")
            self.ui.label_right_gesture.setText("-")
            self.right_warmup_active = False
            self.clear_right_graphs()

        elif name == "left_emg":
            self.ui.label_left_status.setText("disconnected 💤")
            self.ui.label_left_gesture.setText("-")
            self.left_warmup_active = False
            self.clear_left_graphs()

    def poll_logs(self):
        for line in self.pm.get_new_logs():
            self.append_log(line)

    # -------------------------
    # BLE / Warmup / Gesture
    # -------------------------
    def update_right_ble_status(self, status: str):
        if status == "idle":
            self.ui.label_right_status.setText("Idle")
        elif status == "scanning":
            self.ui.label_right_status.setText("scanning...🛰️")
        elif status == "connected":
            self.ui.label_right_status.setText("connected ✅")
        elif status == "disconnected":
            self.ui.label_right_status.setText("disconnected 💤")
            self.ui.label_right_gesture.setText("-")
            self.right_warmup_active = False
        elif status == "error":
            self.ui.label_right_status.setText("error 🚫")

    def update_left_ble_status(self, status: str):
        if status == "idle":
            self.ui.label_left_status.setText("Idle")
        elif status == "scanning":
            self.ui.label_left_status.setText("scanning...🛰️")
        elif status == "connected":
            self.ui.label_left_status.setText("connected ✅")
        elif status == "disconnected":
            self.ui.label_left_status.setText("disconnected 💤")
            self.ui.label_left_gesture.setText("-")
            self.left_warmup_active = False
        elif status == "error":
            self.ui.label_left_status.setText("error 🚫")

    def update_right_warmup(self, remaining: int):
        if remaining > 0:
            self.right_warmup_active = True
            self.ui.label_right_gesture.setText(f"Initializing... {remaining}s")
        else:
            if self.right_warmup_active:
                self.ui.label_right_gesture.setText("Ready")
            self.right_warmup_active = False

    def update_left_warmup(self, remaining: int):
        if remaining > 0:
            self.left_warmup_active = True
            self.ui.label_left_gesture.setText(f"Initializing... {remaining}s")
        else:
            if self.left_warmup_active:
                self.ui.label_left_gesture.setText("Ready")
            self.left_warmup_active = False

    def update_right_gesture(self, gesture: str):
        if not self.right_warmup_active:
            self.ui.label_right_gesture.setText(gesture)

    def update_left_gesture(self, gesture: str):
        if not self.left_warmup_active:
            self.ui.label_left_gesture.setText(gesture)

    # -------------------------
    # Graph updates
    # -------------------------
    def update_right_filtered_signal(self, sample):
        if len(sample) < 8:
            return

        for i in range(8):
            self.right_signal_buffers[i].append(float(sample[i]))
            self._update_curve_with_autoscale(
                self.right_curves[i],
                self.right_signal_buffers[i],
                self.right_plots[i],
            )


    def update_left_filtered_signal(self, sample):
        if len(sample) < 8:
            return

        for i in range(8):
            self.left_signal_buffers[i].append(float(sample[i]))
            self._update_curve_with_autoscale(
                self.left_curves[i],
                self.left_signal_buffers[i],
                self.left_plots[i],
            )


    def clear_right_graphs(self):
        for i in range(8):
            self.right_signal_buffers[i].clear()
            self.right_signal_buffers[i].extend([0.0] * self.graph_buffer_len)
            self._update_curve_with_autoscale(
                self.right_curves[i],
                self.right_signal_buffers[i],
                self.right_plots[i],
            )

    def clear_left_graphs(self):
        for i in range(8):
            self.left_signal_buffers[i].clear()
            self.left_signal_buffers[i].extend([0.0] * self.graph_buffer_len)
            self._update_curve_with_autoscale(
                self.left_curves[i],
                self.left_signal_buffers[i],
                self.left_plots[i],
            )

    def close(self):
        try:
            self.pm.stop_all()
        except Exception:
            pass

        try:
            if hasattr(self, "ros_node"):
                self.ros_node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
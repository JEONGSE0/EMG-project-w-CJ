import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32MultiArray, Float32, Bool
from PySide6.QtCore import QObject, Signal


class RosSignals(QObject):
    right_ble_status = Signal(str)
    left_ble_status = Signal(str)

    right_warmup_remaining = Signal(int)
    left_warmup_remaining = Signal(int)

    right_gesture = Signal(str)
    left_gesture = Signal(str)

    right_filtered_signal = Signal(list)
    left_filtered_signal = Signal(list)

    right_current_speed = Signal(float)
    left_current_speed = Signal(float)


class GuiRosNode(Node):
    def __init__(self, signals: RosSignals):
        super().__init__("emg_system_gui_node")
        self.signals = signals

        # BLE status
        self.create_subscription(
            String,
            "/emg/right/ble_status",
            self._cb_right_ble_status,
            10,
        )
        self.create_subscription(
            String,
            "/emg/left/ble_status",
            self._cb_left_ble_status,
            10,
        )

        # Warmup
        self.create_subscription(
            Int32,
            "/emg/right/warmup_remaining",
            self._cb_right_warmup,
            10,
        )
        self.create_subscription(
            Int32,
            "/emg/left/warmup_remaining",
            self._cb_left_warmup,
            10,
        )

        # Gesture
        self.create_subscription(
            String,
            "/emg/gesture",
            self._cb_right_gesture,
            10,
        )
        self.create_subscription(
            String,
            "/emg/gesture_left",
            self._cb_left_gesture,
            10,
        )

        # Filtered EMG signal
        self.create_subscription(
            Float32MultiArray,
            "/emg/signal_filtered",
            self._cb_right_filtered_signal,
            50,
        )
        self.create_subscription(
            Float32MultiArray,
            "/emg/signal_filtered_left",
            self._cb_left_filtered_signal,
            50,
        )

        self.pub_right_speed_delta = self.create_publisher(
            Float32, "/so101/right/speed_delta", 10
        )
        self.pub_left_speed_delta = self.create_publisher(
            Float32, "/so101/left/speed_delta", 10
        )

        self.pub_right_reset = self.create_publisher(
            Bool, "/so101/right/reset", 10
        )
        self.pub_left_reset = self.create_publisher(
            Bool, "/so101/left/reset", 10
        )

        self.create_subscription(
            Float32,
            "/so101/right/current_speed",
            self._cb_right_current_speed,
            10,
        )
        self.create_subscription(
            Float32,
            "/so101/left/current_speed",
            self._cb_left_current_speed,
            10,
        )

    def _cb_right_ble_status(self, msg: String):
        self.signals.right_ble_status.emit(msg.data)

    def _cb_left_ble_status(self, msg: String):
        self.signals.left_ble_status.emit(msg.data)

    def _cb_right_warmup(self, msg: Int32):
        self.signals.right_warmup_remaining.emit(int(msg.data))

    def _cb_left_warmup(self, msg: Int32):
        self.signals.left_warmup_remaining.emit(int(msg.data))

    def _cb_right_gesture(self, msg: String):
        self.signals.right_gesture.emit(msg.data)

    def _cb_left_gesture(self, msg: String):
        self.signals.left_gesture.emit(msg.data)

    def _cb_right_filtered_signal(self, msg: Float32MultiArray):
        self.signals.right_filtered_signal.emit(list(msg.data[:8]))

    def _cb_left_filtered_signal(self, msg: Float32MultiArray):
        self.signals.left_filtered_signal.emit(list(msg.data[:8]))

    def _cb_right_current_speed(self, msg: Float32):
        self.signals.right_current_speed.emit(float(msg.data))

    def _cb_left_current_speed(self, msg: Float32):
        self.signals.left_current_speed.emit(float(msg.data))

    def send_right_speed_delta(self, delta: float):
        msg = Float32()
        msg.data = float(delta)
        self.pub_right_speed_delta.publish(msg)

    def send_left_speed_delta(self, delta: float):
        msg = Float32()
        msg.data = float(delta)
        self.pub_left_speed_delta.publish(msg)

    def send_right_reset(self):
        msg = Bool()
        msg.data = True
        self.pub_right_reset.publish(msg)

    def send_left_reset(self):
        msg = Bool()
        msg.data = True
        self.pub_left_reset.publish(msg)
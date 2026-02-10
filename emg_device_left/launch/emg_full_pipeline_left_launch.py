#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _spawn_classifier(context):
    lc = lambda name: LaunchConfiguration(name).perform(context)

    conda_prefix = os.environ.get("CONDA_PREFIX", "")
    python_exec = os.path.join(conda_prefix, "bin", "python3") if conda_prefix else "python3"

    params = [
        # input
        "-p", f"input_topic:={lc('input_topic')}",
        "-p", f"num_channels:={lc('num_channels')}",

        # sampling/window/hop
        "-p", f"fs:={lc('fs')}",
        "-p", f"window_size:={lc('window_size')}",
        "-p", f"pred_hop:={lc('pred_hop')}",

        # warmup
        "-p", f"warmup_sec:={lc('warmup_sec')}",

        # filter
        "-p", f"enable_filter:={lc('enable_filter')}",

        # model
        "-p", f"model_path:={lc('model_path')}",
        "-p", f"model_filename:={lc('model_filename')}",

        # decision (3연속 + 0.8s 유지)
        "-p", f"confirm_n:={lc('confirm_n')}",
        "-p", f"hold_hops:={lc('hold_hops')}",

        # outputs
        "-p", f"gesture_topic:={lc('gesture_topic')}",
        "-p", f"gesture_idx_topic:={lc('gesture_idx_topic')}",
        "-p", f"gesture_conf_topic:={lc('gesture_conf_topic')}",
        "-p", f"publish_probs:={lc('publish_probs')}",
        "-p", f"gesture_probs_topic:={lc('gesture_probs_topic')}",

        # misc
        "-p", f"qos_depth:={lc('classifier_qos_depth')}",
        "-p", f"debug:={lc('debug')}",
    ]

    return [
        ExecuteProcess(
            cmd=[python_exec, "-m", "emg_device_left.emg_classifier_node", "--ros-args", *params],
            output="screen",
            name="emg_classifier_node_via_conda",
            env=os.environ,
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("emg_device_left")

    default_model = os.path.join(pkg_share, "models", "re_model_2.joblib")

    # ---------------- BLE args ----------------
    address             = DeclareLaunchArgument("address", default_value="")
    name_filter          = DeclareLaunchArgument("name_filter", default_value="nrf-EMG_left")
    service_uuid         = DeclareLaunchArgument("service_uuid", default_value="12345679-1234-5678-1234-123456789abc")
    emg_char_uuid        = DeclareLaunchArgument("emg_char_uuid", default_value="abcdef13-3456-789a-bcde-f123456789ab")
    emg_topic            = DeclareLaunchArgument("emg_topic", default_value="/emg/signal_left")
    publish_queue        = DeclareLaunchArgument("publish_queue", default_value="50")
    log_found_devices    = DeclareLaunchArgument("log_found_devices", default_value="true")
    drain_period_sec     = DeclareLaunchArgument("drain_period_sec", default_value="0.005")
    publish_per_channel  = DeclareLaunchArgument("publish_per_channel", default_value="false")
    per_channel_prefix   = DeclareLaunchArgument("per_channel_prefix", default_value="")
    per_channel_decim    = DeclareLaunchArgument("per_channel_decimation", default_value="1")

    # ---------------- Classifier args ----------------
    input_topic          = DeclareLaunchArgument("input_topic", default_value=LaunchConfiguration("emg_topic"))
    num_channels         = DeclareLaunchArgument("num_channels", default_value="8")

    # 500 samples = 1s @ 500Hz
    window_size          = DeclareLaunchArgument("window_size", default_value="500")

    # hop=50 samples => 10Hz @ 500Hz
    pred_hop             = DeclareLaunchArgument("pred_hop", default_value="50")

    warmup_sec           = DeclareLaunchArgument("warmup_sec", default_value="10.0")

    enable_filter        = DeclareLaunchArgument("enable_filter", default_value="true")
    fs                   = DeclareLaunchArgument("fs", default_value="500")

    model_path           = DeclareLaunchArgument("model_path", default_value=default_model)
    model_filename       = DeclareLaunchArgument("model_filename", default_value="re_model_2.joblib")

    # 너 조건 반영:
    # - 3연속(confirm_n=3)
    # - 0.8초 유지(10Hz => 8 hops)
    confirm_n            = DeclareLaunchArgument("confirm_n", default_value="3")
    hold_hops            = DeclareLaunchArgument("hold_hops", default_value="8")

    gesture_topic        = DeclareLaunchArgument("gesture_topic", default_value="/emg/gesture_left")
    gesture_idx_topic    = DeclareLaunchArgument("gesture_idx_topic", default_value="/emg/gesture_idx_left")
    gesture_conf_topic   = DeclareLaunchArgument("gesture_conf_topic", default_value="/emg/gesture_confidence_left")
    publish_probs        = DeclareLaunchArgument("publish_probs", default_value="false")
    gesture_probs_topic  = DeclareLaunchArgument("gesture_probs_topic", default_value="/emg/gesture_probs_left")

    classifier_qos       = DeclareLaunchArgument("classifier_qos_depth", default_value="50")
    debug                = DeclareLaunchArgument("debug", default_value="1")

    # ---------------- BLE node ----------------
    ble_node = Node(
        package="emg_device_left",
        executable="emg_node_left",
        name="emg_ble_node_left",
        output="screen",
        parameters=[{
            "address":                LaunchConfiguration("address"),
            "name_filter":            LaunchConfiguration("name_filter"),
            "service_uuid":           LaunchConfiguration("service_uuid"),
            "emg_char_uuid":          LaunchConfiguration("emg_char_uuid"),
            "topic":                  LaunchConfiguration("emg_topic"),
            "publish_queue":          LaunchConfiguration("publish_queue"),
            "log_found_devices":      LaunchConfiguration("log_found_devices"),
            "drain_period_sec":       LaunchConfiguration("drain_period_sec"),
            "publish_per_channel":    LaunchConfiguration("publish_per_channel"),
            "per_channel_prefix":     LaunchConfiguration("per_channel_prefix"),
            "per_channel_decimation": LaunchConfiguration("per_channel_decimation"),
        }],
    )

    # classifier: conda python으로 실행
    classifier_proc = OpaqueFunction(function=_spawn_classifier)

    return LaunchDescription([
        # ---- declare (BLE) ----
        address, name_filter, service_uuid, emg_char_uuid, emg_topic,
        publish_queue, log_found_devices, drain_period_sec,
        publish_per_channel, per_channel_prefix, per_channel_decim,

        # ---- declare (Classifier) ----
        input_topic, num_channels,
        window_size, pred_hop, warmup_sec,
        enable_filter, fs,
        model_path, model_filename,
        confirm_n, hold_hops,
        gesture_topic, gesture_idx_topic, gesture_conf_topic,
        publish_probs, gesture_probs_topic,
        classifier_qos, debug,

        # ---- nodes ----
        ble_node, classifier_proc,
    ])
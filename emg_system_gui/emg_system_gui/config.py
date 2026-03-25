ROS_SETUP = "source /opt/ros/jazzy/setup.bash"
WS_SETUP = "source ~/ros2_ws/install/setup.bash"
CONDA_ENV = "test"

SO101_REAL_DEMO_DIR = "~/ros2_ws/src/so101/gkim451/demo"

COMMANDS = {
    "connect_right_emg": "ros2 launch emg_device emg_full_pipeline_launch.py name_filter:=nrf",
    "connect_left_emg": "ros2 launch emg_device_left emg_full_pipeline_left_launch.py name_filter:=nrf",

    "shadow_right": "ros2 run emg_shadow_hand_bridge shadow_hand_gesture_control",
    "shadow_left": "ros2 run emg_shadow_hand_bridge shadow_hand_gesture_control_left",

    "so101_left": "ros2 run so101 so101",
    "so101_right": "ros2 run so101 so101_right",

    "so101_real_follow_left": "ros2 run so101 so101_real_control",
    "so101_real_follow_right": "ros2 run so101 so101_real_control_right",

    "real_robot_left": (
        f"cd {SO101_REAL_DEMO_DIR} && "
        "python ./run.py --port /dev/follower_0 --id lerobot_follower_0 --cmd cmd.json --hz 30"
    ),
    "real_robot_right": (
        f"cd {SO101_REAL_DEMO_DIR} && "
        "python ./run.py --port /dev/follower_1 --id lerobot_follower_1 --cmd cmd1.json --hz 30"
    ),
}
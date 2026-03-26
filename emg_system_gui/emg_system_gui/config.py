ROS_SETUP = "source /opt/ros/jazzy/setup.bash"
WS_SETUP = "source ~/ros2_ws/install/setup.bash"

COMMANDS = {
    "connect_right_emg": {
        "env": "test",
        "cwd": "~/ros2_ws",
        "command": "ros2 launch emg_device emg_full_pipeline_launch.py name_filter:=nrf",
    },
    "connect_left_emg": {
        "env": "test",
        "cwd": "~/ros2_ws",
        "command": "ros2 launch emg_device_left emg_full_pipeline_left_launch.py name_filter:=nrf",
    },

    "shadow_right": {
        "env": "test",
        "cwd": "~/ros2_ws",
        "command": "ros2 run emg_shadow_hand_bridge shadow_hand_gesture_control",
    },
    "shadow_left": {
        "env": "test",
        "cwd": "~/ros2_ws",
        "command": "ros2 run emg_shadow_hand_bridge shadow_hand_gesture_control_left",
    },

    "so101_left": {
        "env": "test",
        "cwd": "~/ros2_ws",
        "command": "ros2 run so101 so101",
    },
    "so101_right": {
        "env": "test",
        "cwd": "~/ros2_ws",
        "command": "ros2 run so101 so101_right",
    },

    "so101_real_follow_left": {
        "env": "test",
        "cwd": "~/ros2_ws",
        "command": (
            "ros2 run so101 so101_real_control "
            "--ros-args -p out_path:=/home/meow/ros2_ws/src/so101/gkim/demo/cmd.json"
        ),
    },

    "so101_real_follow_right": {
        "env": "test",
        "cwd": "~/ros2_ws",
        "command": (
            "ros2 run so101 so101_real_control_right "
            "--ros-args -p out_path:=/home/meow/ros2_ws/src/so101/gkim/demo/cmd1.json"
        ),
    },

    "real_robot_left": {
        "env": "base",
        "cwd": "~/ros2_ws/src/so101/gkim/demo",
        "command": "python ./run.py --port /dev/follower_0 --id lerobot_follower_0 --cmd cmd.json --hz 30",
    },
    "real_robot_right": {
        "env": "base",
        "cwd": "~/ros2_ws/src/so101/gkim/demo",
        "command": "python ./run.py --port /dev/follower_1 --id lerobot_follower_1 --cmd cmd1.json --hz 30",
    },
}
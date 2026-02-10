# EMG project w/CJ
Robot Control Based on EMG Signals in the Isaac Sim Environment 🤖

### Default Setup 🚀
#### 1. Conda Environment
Environment name: 'test'

``` bash
$ conda activate test
```

#### 2. ROS2 setup
``` bash
$ source /opt/ros/jazzy/setup.bash
```

- If you have updated the ROS2 code, run the following commands in your ros2_ws workspace:
``` bash
$ colcon build
$ source install/setup.bash
```
### Isaac sim Setup 🦾
#### 1. Conda Environment
Environment name: 'env_isaacsim'
``` bash
$ conda activate env_isaacsim
```

#### 2. Start Isaac Sim
```
$ cd isaacsim
$ isaacsim
or
$ ./isaac-sim.sh
```


### Package Description 🧑‍💻
#### 1. emg_connect
- Connects to the EMG device and retrieves EMG signals
- Developed by CJ 🥸
- Run Python File
: main_gui.py
![alt text](image.png)

#### 2. emg_device


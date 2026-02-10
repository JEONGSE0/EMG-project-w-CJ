# EMG project w/CJ
Robot Control Based on EMG Signals in the Isaac Sim Environment

#### Default Setup
- conda environment: test

``` bash
$ conda activate test
```

- ROS2 setup
``` bash
$ source /opt/ros/jazzy/setup.bash
```

- If you updated code(ROS2), you should enter the next command in your workspace(ros2_ws)
``` bash
$ colcon build
$ source install/setup.bash
```
#### Isaac sim Setup
- conda enviroment: env_isaacsim
``` bash
$ conda activate env_isaacsim
$ cd isaacsim

# start isaac sim
$ isaacsim
or
$ ./isaac-sim.sh
```



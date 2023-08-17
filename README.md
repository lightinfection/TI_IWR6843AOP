# TI_IWR6843AOP_ROS2

By ros2 rviz2:

<img src="./ti_test_py/img/rviz2.gif" width="50%" height="50%">

By visualizer.py:

<img src="./ti_test_py/img/animation.gif" width="50%" height="50%">

Refered to: 
```sh
git clone https://git.ti.com/git/mmwave_radar/mmwave_ti_ros.git
```

Another example: https://github.com/nhma20/iwr6843aop_pub

A different method is used when parsing serial port data, and more pointclouds can be acquired without instant failure.

<img src="./ti_test_py/img/benchmark/screenshot.png">

```chrome://tracing/``` is used to visualize threads, and the json for demo is located in the benchmark folder. However, the function is not neccessary when using the sensor. Ignore the benchmark module whenever you want.

### Enviornment

- ROS2 (Ubuntu 22.04 & humble)
- Python3 (3.10.12)
- IWR6843AOPEVM (ES2) mmWave radar device flashed with out-of-box firmware

### Installation

1. Clone the repo to workspace
   ```sh
   cd ~/${workspace}/src/
   git clone https://github.com/nhma20/iwr6843aop_pub.git
   ```
2. Colcon build package
   ```sh
   cd ~/${workspace}
   colcon build --packages-select ti_test_py
   ```
3. Revise parameters in the launch file

    <img src="./ti_test_py/img/defined.png" width="75%" height="75%">

4. Start
    ```sh
    source ./install/setup.bash
    ros2 launch ti_test_py 6843aop_3d_0.launch.py
    ```

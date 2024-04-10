# TI_IWR6843AOP

Now the official driver version for ros2 has been released: 

git://git.ti.com/mmwave_radar/mmwave_ti_ros.git

## Enviornment

- IWR6843AOPEVM (ES2) mmWave radar device flashed with out-of-box firmware
- Python3 (3.10.12)
- Local ROS2 Desktop(Ubuntu 22.04 & humble), or docker (docker-compose version > 3.0)

<!-- A [dockerfile](./ros2_rviz_docker/humble_docker/Dockerfile) is provided for building a container where mmwave can run. -->

## Installation

### Local
1. Clone to your workspace
   ```sh
   git clone https://github.com/lightinfection/TI_IWR6843AOP.git
   ```
2. Build package (Using GDB to build object_detection package if cmake-args is declared as below)
   ```sh
   colcon build --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' --packages-select object_detection
   colcon build --packages-select ti_ros2_driver
   source install/setup.bash
   ```

### Docker (Optional)
1. Install [docker](https://docs.docker.com/engine/install/)

## Use the mmWave sensor in local ROS2

### Single-frame
1. Plugin mmWave sensor and start (if using debug mode, please load the output [log](./src/ti_ros2_driver/debug/result.json) to chrome://tracing/ for log visualization). If the object_detection examples are expected to run, please ignore this step and directly execute the commands in step 2, in which the launch file of this part is included.
    ```sh
    ## Only output detected pointcloud (x,y,z + snr + radial_velocity)
    ros2 launch ti_ros2_driver 6843aop_3d_0.launch.py
    ## Output pointcloud with real-time Range-Doppler and Range-Azimuth Heatmap
    ros2 launch ti_ros2_driver 6843aop_3d_1.launch.py
    ```
    #### **ROS parameters**

    * `cfg_path` - [string]  
    Sets the configure file for ti mmwave, which is stored in the cfg folder    
    Default value: `base.cfg` for 6843aop_3d_0.launch.py, `RADheatmap.cfg` for 6843aop_3d_1.launch.py
    * `command_port` - [string]  
    Command port number for ti  
    Default value: `/dev/ttyUSB0`
    * `data_port` - [string]  
    Data port number for ti     
    Default value: `/dev/ttyUSB1`
    * `frame_id` - [string]  
    The frame id of published PointCloud2 sensor msgs   
    Default value: `ti_mmwaver_0`
    * `topic` - [string]  
    The topic name of published PointCloud2 sensor msgs     
    Default value: `ti_mmwave_0` 
    * `output_RD_heatmap` - [bool]  
    If output real-time Range-Doppler heatmap   
    Default value: `False` for 6843aop_3d_0.launch.py, `True` for 6843aop_3d_1.launch.py
    * `output_RA_heatmap` - [bool]  
    If output real-time Range-Azimuth heatmap   
    Default value: `False` for 6843aop_3d_0.launch.py, `True` for 6843aop_3d_1.launch.py
    * `debug_mode` - [bool]  
    If use debug mode or not during launching     
    Default value: `False`
    * `debug_log_path` - [string]  
    If using debug mode, the path of output debug log   
    Default value: `#{workdir}/install/ti_ros2_driver/share/ti_ros2_driver/debug/result.json`

    #### **Debug mode**
    <img src="./img/debug.png" width="75%" height="75%">

    #### **Real time heatmap**
    A substitute for ti official Out-Of-Box demo visualizer        
    <img src="./img/heatmap.png" width="75%" height="75%">
    
2. Filters for mmWave single frame signals (Examples, including the mmwave sensor bringup in step 1)
    ```sh
    ## passthrough filters + statiscal outlier removal, defaultly use `6843aop_3d_1.launch.py`, which outputs pointcloud with heatmap
    ros2 launch object_detection filter_for_ti_static.launch.py
    ## noise removal by dbscan clustering, defaultly use `6843aop_3d_0.launch.py`, which only outputs pointcloud
    ros2 launch object_detection filter_for_dbscan_cluster.launch.py
    ```
3. Check outcomes on RVIZ

    <img src="./img/single.png" width="75%" height="75%">
    <img src="./img/single_cluster.png" width="75%" height="75%">

### Multi-frames
1. Prepare mmWave multiple frames data (raw pointcloud saved in either pcd or octomap)

2. Post Processing
    ```sh
    ## filter(Optional) + difference of L2 normal extraction

    ## filter: statistical outlier removal
    ros2 launch object_detection filter_for_ti_dynamic.launch.py
    ## filter: dbscan cluster
    ros2 launch object_detection dbscan_cluster_map.launch.py
    ```
    Statistical Outlier Removal + DON

    <img src="./img/multi.png" width="75%" height="75%">

    DBSCAN + DON

    <img src="./img/multi_cluster.png" width="75%" height="75%">

## Launch the mmWave sensor in docker

### Single-frame
1. Plugin mmWave sensor and start, then a docker image named "ti_humble" will be built. Make sure that your host can accept X forwarded connections for displaying rviz.
    ```sh
    cd docker
    ## passthrough filters + statiscal outlier removal
    sudo docker compose --profile single_frame up --build
    ## noise removal by dbscan clustering
    sudo docker compose --profile single_frame_dbscan up --build
    ```
### Multi-frames
1. Make sure that your host can accept X forwarded connections for displaying rviz.
    ```sh
    cd docker
    ## Statistical Outlier Removal + DON
    sudo docker compose --profile map up --build
    ## DBSCAN + DON
    sudo docker compose --profile map_dbscan up --build
    ```
2. Prepare your own map and save it to the [map folder](./src/object_detection/map/), or raw code is revised:
    ```sh
    zip -r src.zip src/ && mv src.zip ./docker/
    cd docker
    ## repeat docker compose build above
    ```

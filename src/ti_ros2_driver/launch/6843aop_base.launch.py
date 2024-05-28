from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    return LaunchDescription([
        Node(
        package="ti_ros2_driver",
        namespace="ti_radar_0",
        executable="pcl_pub",
        parameters=[
            {'cfg_path': str(get_package_share_path('ti_ros2_driver') / 'cfg/base.cfg')},                 # cfg file for ti mmwave, please refer to the /cfg/rules.txt for detail
            {'command_port': '/dev/ttyUSB0'},                                                             # if not known, run "ll/dev/serial/by-id" in terminal
            {'data_port': '/dev/ttyUSB1'},                                                                # if not known, run "ll/dev/serial/by-id" in terminal
            {'frame_id': 'ti_mmwaver_0'},                                                                 # frame id of published topic
            {'topic': 'ti_mmwave_0'},                                                                     # published topic name
            {'roi': '-0.5_0.5; 0.2_10.0; -0.2_1.0'},                                                      # Optional, 3d_ROI_Cube, x: -0.5 ~ 0.5, y: 0.2 ~ 10.0; z: -0.2 ~ 1.0, channel seperatd by ";" while the lower limit and upper limit are seperated by "_"
            {'publish_target_tracker': False},                                                            # Optional, if publish the target tracker as compact pointclouds, only available for 3d_tracking binary mode
            {'output_RD_heatmap': False},                                                                 # Optional, if output Range-Doppler heatmap, only available for out-of-box binay mode
            {'output_RA_heatmap': False},                                                                 # Optional, if output Range-Azimuth heatmap, only available for out-of-box binay mode
            {"debug_mode": False},                                                                        # Optional, if using debug mode
            {"debug_log_path": str(get_package_share_path('ti_ros2_driver') / 'debug/result.json')},      # Optional, if using debug mode, the path of output debug log
        ],
        output="screen",
        emulate_tty=True
        )
    ])
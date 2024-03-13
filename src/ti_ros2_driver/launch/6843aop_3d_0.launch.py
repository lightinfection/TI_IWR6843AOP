from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
        package="ti_ros2_driver",
        namespace="ti_radar_0",
        executable="pcl_pub",
        parameters=[
            {'cfg_path': os.getcwd()+'/src/ti_ros2_driver/cfg/staticRetention.cfg'},      # cfg file for ti mmwave
            {'command_port': '/dev/ttyUSB0'},                                             # if not known, run "ll/dev/serial/by-id" in terminal
            {'data_port': '/dev/ttyUSB1'},                                                # if not known, run "ll/dev/serial/by-id" in terminal
            {'frame_id': 'ti_mmwaver_0'},                                                 # frame id of published topic
            {'topic': 'ti_mmwave_0'},                                                     # published topic name
            {"debug_mode": False}                                                         # if using debug mode
        ],
        output="screen",
        emulate_tty=True
        )
    ])
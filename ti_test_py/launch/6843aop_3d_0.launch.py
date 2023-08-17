from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
        package="ti_test_py",
        namespace="ti_radar_0",
        executable="pcl_pub",
        parameters=[
            {'cfg_path': os.getcwd()+'/src/TI_IWR6843AOP_ROS2/ti_test_py/cfg/staticRetention.cfg'},
            {'command_port': '/dev/ttyUSB0'},
            {'data_port': '/dev/ttyUSB1'},
            {'frame_id': 'ti_mmwaver_0'},
            {'topic': 'ti_mmwave_0'}
        ],
        output="screen",
        emulate_tty=True
        )
    ])
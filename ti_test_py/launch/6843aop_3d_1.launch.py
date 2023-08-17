from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
        package="ti_test_py",
        namespace="ti_radar_1",
        executable="pcl_pub",
        parameters=[
            {'cfg_path': os.getcwd()+'/src/TI_IWR6843AOP_ROS2/ti_test_py/cfg/staticRetention.cfg'},
            {'command_port': '/dev/ttyUSB2'},
            {'data_port': '/dev/ttyUSB3'},
            {'frame_id': 'ti_mmwaver_1'},
            {'topic': 'ti_mmwave_1'}
        ],
        output="screen",
        emulate_tty=True
        )
    ])
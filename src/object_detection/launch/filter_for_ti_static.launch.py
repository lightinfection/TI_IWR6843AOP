from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(str(get_package_share_path('ti_ros2_driver') / 'launch/6843aop_Heatmap.launch.py')),
        Node(
            package="object_detection",
            executable="filter_single_frame",
            name="filter_single_frame",
            parameters=[{
                 "x": True, "reverse_x": False, "x_min": 0.1, "x_max": 5.0,            # x-channel, if reverse the filtered output, x_min value, x_max value
                 "y": True, "reverse_y": False, "y_min": -0.3, "y_max": 0.3,           # y-channel
                 "z": True, "reverse_z": False, "z_min": -0.1, "z_max": 2.0,           # z-channel
                 "i": True, "reverse_i": False, "i_min": 8.0, "i_max": 100.0,          # intensity(SNR)-channel 
                 "SOR": True, "reverse_sor": False, "stddev": 0.005, "mean_K": 10.0,   # Statistical Outlier Removal Filter
            }],
            remappings=[
                ("/ti_mmwave/radar_scan_pcl","/ti_radar_0/ti_mmwave_0")
            ],
            output="screen",
            emulate_tty=True
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', str(get_package_share_path('object_detection') / 'rviz/mmwave_single_frame.rviz')],
        ),
    ])
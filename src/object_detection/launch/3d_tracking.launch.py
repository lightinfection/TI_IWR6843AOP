from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(str(get_package_share_path('ti_ros2_driver') / 'launch/6843aop_3d_tracking.launch.py')),
        Node(
            package="object_detection",
            executable="dbscan",
            name="dbscan",
            parameters=[{
                "eps": 0.8,                   # The maximum distance between two samples for one to be considered as in the neighborhood of the other.
                "minimum_points": 15,         # The number of samples in a neighborhood for a point to be considered as a core point.
                "bounding_boxes": False,       # if show bounding box on RVIZ
                "target_frame": "/ti_mmwaver_0",  # target frame id of bounding box
                "name_box": "dbscan",         # namespace of bounding box
            }],
            remappings=[
                ("/ti_mmwave/radar_scan_pcl","/ti_radar_0/ti_mmwave_0")
            ],
            output="screen",
            emulate_tty=True
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', str(get_package_share_path('object_detection') / 'rviz/3d_tracking.rviz')],
        ),
    ])
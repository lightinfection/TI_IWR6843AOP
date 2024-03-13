from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="object_detection",
            executable="dbscan",
            name="dbscan",
            parameters=[{
                "eps": 0.8,                   # The maximum distance between two samples for one to be considered as in the neighborhood of the other.
                "minimum_points": 15,         # The number of samples in a neighborhood for a point to be considered as a core point.
                "bounding_boxes": True,       # if show bounding box on RVIZ
                "target_frame": "/ti_mmwaver_0",  # target frame id of bounding box
                "name_box": "dbscan",         # namespace of bounding box
            }],
            remappings=[
                ("/ti_mmwave/radar_scan_pcl","/ti_radar_0/ti_mmwave_0")
            ],
            output="screen",
            emulate_tty=True
        )
    ])
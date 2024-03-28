from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    default_map = str(get_package_share_path('object_detection') / 'map/mmwave.bt')
    print("MAP: " + default_map)
    return LaunchDescription([
        Node(
            package="object_detection",
            executable="filter_multi_frames",
            name="filter_multi_frames",
            parameters=[{
                 "map": default_map, "isOrganized": False,         #   Map path, if pointcloud in the map is organized or not;
                #  "filter": 1, "stddev": 0.04, "mean_K": 20,        #   StatisticalOutlierRemoval = 1, (recommended)
                #  "filter": 2, "radius": 0.1, "mean_K": 10,       #   RadiusOutlierRemoval = 2, 
                 "filter": 3, "radius": 0.1, "mean_K": 20,       #   DBSCAN = 3,
                #  "filter": 4, "lx": 0.1, "ly": 0.1, "lz": 0.01,  #   VoxelGrid = 4, (recommended)
                 "DON": True, "threshold": 0.16, "lower_limit": 0.10, "upper_limit": 0.25,
            }],
            # remappings=[],
            output="screen",
            emulate_tty=True
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', str(get_package_share_path('object_detection') / 'rviz/mmwave_multi_frames_dbscan.rviz')],
        ),
    ])
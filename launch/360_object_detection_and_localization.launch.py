import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory of each subpackage
    depth_map_pkg_dir = get_package_share_directory('ros2_depth_map_detection_localization_cpp')
    lidar_camera_pkg_dir = get_package_share_directory('ros2_lidar_camera_fusion_with_detection_cpp')

    # Paths to the package launch files
    depth_map_launch_path = os.path.join(
        depth_map_pkg_dir, 
        'launch', 
        'depth_map_detection_localization_yolo.launch.py'
    )
    
    lidar_camera_launch_path = os.path.join(
        lidar_camera_pkg_dir, 
        'launch', 
        'lidar_camera_fusion_yolo.launch.py'
    )

    # Create a launch description
    ld = LaunchDescription()

    # Include the launch files from the two packages
    try:
        # For depth map launch file
        if os.path.exists(depth_map_launch_path):
            ld.add_action(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(depth_map_launch_path)
                )
            )
        else:
            print(f"Warning: Launch file not found at {depth_map_launch_path}")

        # For lidar camera launch file
        if os.path.exists(lidar_camera_launch_path):
            ld.add_action(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(lidar_camera_launch_path)
                )
            )
        else:
            print(f"Warning: Launch file not found at {lidar_camera_launch_path}")

    except Exception as e:
        print(f"Error including launch files: {e}")

    return ld

# ROS2 360° Object Detection and Localization Packages

## 📝 Description

This repository contains two complementary ROS2 packages designed for advanced multi-sensor object detection and localization:

1. **Lidar-Camera Fusion Package**
2. **Depth Map Detection and Localization Package**

These packages provide a comprehensive solution for 360° object detection and localization using advanced sensor fusion techniques, leveraging LiDAR point clouds, depth maps, and RGB data.

## 🚀 Features

### Lidar-Camera Fusion Package
- Dynamic transform handling between sensor frames
- 3D position estimation within object bounding boxes
- Real-time point cloud projection onto camera images
- Multi-object detection and localization

### Depth Map Detection Package
- Point cloud to depth map conversion
- Object-specific depth map generation
- 3D pose estimation for detected objects
- Multi-object real-time tracking

## 🛠️ Technologies

- ROS2 Humble
- C++
- PCL (Point Cloud Library)
- OpenCV
- YOLOv8 for object detection

## 📦 Prerequisites

- ROS2 Humble
- YOLOvX ROS package
- C++ compiler (GCC 8+)
- PCL and OpenCV

## 🔧 Installation

### Install Dependencies
```bash
sudo apt-get update
sudo apt-get install libpcl-dev libopencv-dev
```
### Clone Repositories
```bash
cd ~/ros2_ws/src

```

### Build Packages
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_lidar_camera_fusion_with_detection_cpp ros2_depth_map_detection_localization_cpp
source install/setup.bash
```

## 🚀 Usage

# ROS2 360° Object Detection and Localization Packages

## 📝 Description

This repository contains two complementary ROS2 packages designed for advanced multi-sensor object detection and localization:

1. **Lidar-Camera Fusion Package**
2. **Depth Map Detection and Localization Package**

These packages provide a comprehensive solution for 360° object detection and localization using advanced sensor fusion techniques, leveraging LiDAR point clouds, depth maps, and RGB data.

## 🚀Key Features

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

## 🖼️ Demonstrations

### Comprehensive Sensor Fusion Workflow

Our integrated system provides a complete solution for object detection and localization:

#### Lidar-Camera Fusion
<p align="center">
  <img src="lidar_camera_fusion/images/Camera_Lidar_Fusion.gif" alt="Lidar-Camera Fusion" width="500"/>
  <br>
  <strong>Key Visualization:</strong> Lidar points overlaid on camera image, showing precise object localization
</p>

##### Lidar-Camera Fusion Capabilities
- **Point Cloud Projection**: Accurately maps LiDAR points onto camera frames
- **Object Boundary Tracking**: Highlights detected objects with point cloud data
- **Real-time Processing**: Provides instant 3D object positioning

#### Depth Map Detection
<p align="center">
  <img src="depth_map_detection/images/3.gif" alt="Depth Map Detection" width="500"/>
  <br>
  <strong>Key Visualization:</strong> Depth map generation and object localization
</p>

##### Depth Map Detection Capabilities
- **Scene Depth Mapping**: Converts point clouds to comprehensive depth representations
- **Object-Specific Depth Isolation**: Highlights depths for detected objects
- **3D Pose Estimation**: Calculates precise object positions

### Integrated Detection Workflow
1. **LiDAR Point Cloud Acquisition**
2. **Camera Image Processing**
3. **Object Detection with YOLO**
4. **Depth Map Generation**
5. **3D Position Estimation**
6. **Multi-Object Tracking**

## 🛠️ Technologies

- **Framework**: ROS2 Humble
- **Programming**: C++
- **Libraries**: 
  - PCL (Point Cloud Library)
  - OpenCV
  - YOLOv8 for object detection

## (Rest of the previous README remains the same)

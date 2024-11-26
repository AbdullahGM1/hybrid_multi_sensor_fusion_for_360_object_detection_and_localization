# ROS2 360° Multi-Sensor Object Detection and Localization 🤖📡

## Overview 🌐

This repository provides a cutting-edge ROS2 solution for comprehensive 360° object detection and localization, leveraging advanced sensor fusion techniques. By integrating LiDAR point clouds, depth maps, and RGB imagery, the packages offer robust and real-time object tracking and positioning.

## 🚀 Key Features

### Lidar-Camera Fusion Package
- **Dynamic Sensor Fusion**: Seamless integration of LiDAR and camera data
- **Real-Time 3D Localization**: Precise object position estimation
- **Point Cloud Projection**: Accurate mapping of LiDAR points onto camera frames
- **Multi-Object Detection**: Simultaneous tracking of multiple objects

### Depth Map Detection Package
- **Intelligent Depth Mapping**: Convert point clouds to comprehensive depth representations
- **Object-Specific Depth Analysis**: Isolate and highlight object depths
- **3D Pose Estimation**: Calculate precise object positions and orientations
- **Real-Time Tracking**: Continuous object monitoring

## 🖼️ Visualizations

### Sensor Fusion Workflow
| Lidar-Camera Fusion | Depth Map Detection |
|:---:|:---:|
| ![Lidar-Camera Fusion](images/Camera_Lidar_Fusion.gif) | ![Depth Map Detection](images/3.gif) |
| **Precise Object Localization** | **Depth-Based Object Detection** |

## 🛠️ Technical Specifications

### Technologies
- **Framework**: ROS2 Humble
- **Programming**: C++
- **Key Libraries**:
  - Point Cloud Library (PCL)
  - OpenCV
  - YOLOv8 for Object Detection

### System Architecture
1. LiDAR Point Cloud Acquisition
2. Camera Image Processing
3. YOLO Object Detection
4. Depth Map Generation
5. 3D Position Estimation
6. Multi-Object Tracking

## 📋 Prerequisites

### System Requirements
- ROS2 Humble ([Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- YOLOvX ROS ([Setup Instructions](https://github.com/mgonzs13/yolov8_ros))
- GCC 8 or newer
- Dependencies: PCL, OpenCV

### Dependency Installation
```bash
sudo apt-get update
sudo apt-get install libpcl-dev libopencv-dev
```

## 🔧 Installation & Setup

### Clone Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/AbdullahGM1/hybrid_multi_sensor_fusion_for_360_object_detection_and_localization.git
```

### Build Packages
```bash
cd ~/ros2_ws
colcon build --packages-select hybrid_multi_sensor_fusion_for_360_object_detection_and_localization
source install/setup.bash
```

### Launch
```bash
ros2 launch hybrid_multi_sensor_fusion_for_360_object_detection_and_localization 360_object_detection_and_localization.launch.py
```

## 🔬 Detailed Node Information

### Lidar-Camera Fusion Node

#### Subscribed Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/scan/points` | `sensor_msgs/PointCloud2` | Raw LiDAR data |
| `/interceptor/gimbal_camera` | `sensor_msgs/Image` | Camera stream |
| `/yolo/tracking` | `vision_msgs/Detection2DArray` | Object detections |

#### Published Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/image_lidar` | `sensor_msgs/Image` | LiDAR-annotated image |
| `/detected_object_distance` | `std_msgs/Float32MultiArray` | Object distances |

### Depth Map Detection Node

#### Subscribed Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/scan/points` | `sensor_msgs/PointCloud2` | Raw LiDAR data |
| `/yolo/tracking` | `vision_msgs/Detection2DArray` | Object detections |

#### Published Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/original_depth_map` | `sensor_msgs/Image` | Scene depth map |
| `/detected_objects_pose` | `geometry_msgs/PoseArray` | Object 3D poses |

## 🤝 Contributing

Contributions are welcome! Please:
- Open issues for bugs or feature requests
- Submit pull requests with improvements
- Follow existing code style and documentation practices

## 📜 License

MIT

# Detector_desktop  
这是一个应用于桌面的 检测-抓取 程序。本程序直接从Realsense相机读取RGB-D图像。对采集的图像进行目标检测，其中
目标检测算法是基于Libtorch或ncnn实现的YOLOv5。根据目标检测的结果，重建每个物体的点云，然后计算其抓取位姿。
最后将抓取位姿发送给Kinova机械臂的驱动，实现目标的抓取。  
整个程序分为4个线程，包括：Realsense相机读取线程、目标检测线程、点云重建和滤波线程、机械臂抓取线程。  
测试视频在 [这里](https://www.bilibili.com/video/BV1TS4y1j7AV) 。  
![test](https://github.com/chenjianqu/detector_desktop/blob/master/config/test.png)


## Requirement
* OpenCV3
* PCL1.8
* realsense2
* Eigen3
* Libtorch
* NCNN (Optimal)
* ROS
* Kinova Driver

## Quick Start
### 1.下载yolov5
安装[yolov5](https://github.com/ultralytics/yolov5)，运行成功，并导出其yolov5的torchscript模型。

### 2.下载本项目
```shell
git clone https://github.com/chenjianqu/detector_desktop.git
```
并将detector_desktop放到ROS工作空间中。
### 3.编译运行
修改CMakeLists.txt，确保libtorch和ncnn的路径正确。  
修改config.yaml，确保其中各个选项设置正确。
```shell
catkin_make -j10

rosrun detector_desktop detector_desktop_ros xxx/config.yaml
或
rosrun detector_desktop detector_desktop xxx/config.yaml
```
注：在运行本程序之前，你必须先运行roscore、Kinova ROS Driver等程序。 



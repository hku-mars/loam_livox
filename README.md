# Livox-LOAM
## A robust LiDAR Odemetry and Mapping (LOAM) package for Livox-LiDAR

<img src="https://github.com/ziv-lin/livox_loam/blob/for_opensource/pics/demo_cyt.gif" width = 640 height = 480 />

This code is modified from LOAM, [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED) and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).

**Modification:**

1. Feature extraction for Livox-LiDAR sensor.
2. Noise, outliers, dynamic objects  rejection
3. Interpolartation and motion deblur.

**Developer:** [Jiarong Lin](https://github.com/ziv-lin)


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).


## 2. Build
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/A-LOAM.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
## 3. Directly run
Connect to your PC to Livox LiDAR (mid-40) by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```
    roslaunch livox_loam livox.launch
```

## 4. Rosbag Example
### 4.1. **Common rosbag**
Download [Our recorded rosbag](https://drive.google.com/drive/folders/1HWomWWPSEVvka2QVB2G41iRvSwIt5NWf?usp=sharing) and than
```
roslaunch livox_loam rosbag.launch
rosbag play YOUR_DOWNLOADED.bag
```
<div align="center">
    <img src="https://github.com/ziv-lin/livox_loam/blob/for_opensource/pics/CYT_01.png" width=45% /><img src="https://github.com/ziv-lin/livox_loam/blob/for_opensource/pics/CYT_02.png" width=45% />
</div>

### 4.1. **Large-scale rosbag**
For large scale rosbag (For example [HKUST_01.bag](https://drive.google.com/file/d/1OoAu0WcRhyDsQB9ltPLWeZ2WZlGJz6LO/view?usp=sharing) ), we recommand you launch with bigger line and plane resolution (using *rosbag_largescale.launch*)
```
roslaunch livox_loam rosbag_largescale.launch
rosbag play YOUR_DOWNLOADED.bag
```
<div align="center">
    <img src="https://github.com/ziv-lin/livox_loam/blob/for_opensource/pics/HKUST_01.png" width=45% >
    <img src="https://github.com/ziv-lin/livox_loam/blob/for_opensource/pics/HKUST_02.png" width=45% >
</div>

## 5.Acknowledgements
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) , [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).


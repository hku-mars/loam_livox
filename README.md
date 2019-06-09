# LOAM-Livox
## A robust LiDAR Odemetry and Mapping (LOAM) package for Livox-LiDAR

<img src="https://github.com/hku-mars/loam_livox/blob/master/pics/demo_cyt.gif" width = 100% />

**Loam-Livox** is a robust, low drift, and real time odometry and mapping package for [*Livox LiDARs*](https://www.livoxtech.com/), significant low cost and high performance LiDARs that are designed for massive industrials uses. Our package address many key issues: feature extraction and selection in a very limited FOV, robust outliers rejection, moving objects filtering, and motion distortion compensation. We use [*Ceres-Solver*](http://ceres-solver.org/) for scan matching to avoid complicated differential geometry derivation. The codes are well structured and streamlined to improve readability and extendability.

In the development of our package, we reference to LOAM, [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED), and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).


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
    git clone https://github.com/hku-mars/loam_livox.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
## 3. Directly run
Connect to your PC to Livox LiDAR (mid-40) by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```
    roslaunch loam_livox livox.launch
```

## 4. Rosbag Example
### 4.1. **Common rosbag**
Download [Our recorded rosbag](https://drive.google.com/drive/folders/1HWomWWPSEVvka2QVB2G41iRvSwIt5NWf?usp=sharing) and then
```
roslaunch loam_livox rosbag.launch
rosbag play YOUR_DOWNLOADED.bag
```
<div align="center">
    <img src="https://github.com/hku-mars/loam_livox/blob/master/pics//CYT_01.png" width=45% >
    <img src="https://github.com/hku-mars/loam_livox/blob/master/pics//CYT_02.png" width=45% >
</div>

<div align="center">
    <img src="https://github.com/hku-mars/loam_livox/blob/master/pics//HKU_ZYM_01.png" width=45% >
    <img src="https://github.com/hku-mars/loam_livox/blob/master/pics//HKU_ZYM_02.png" width=45% >
</div>

### 4.1. **Large-scale rosbag**
For large scale rosbag (For example [HKUST_01.bag](https://drive.google.com/file/d/1OoAu0WcRhyDsQB9ltPLWeZ2WZlGJz6LO/view?usp=sharing) ), we recommand you launch with bigger line and plane resolution (using *rosbag_largescale.launch*)
```
roslaunch loam_livox rosbag_largescale.launch
rosbag play YOUR_DOWNLOADED.bag
```
<div align="center">
    <img src="https://github.com/hku-mars/loam_livox/blob/master/pics/HKUST_01.png" width=45% >
    <img src="https://github.com/hku-mars/loam_livox/blob/master/pics/HKUST_02.png" width=45% >
</div>

## 5.Acknowledgments
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) LOAM, [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED), and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).

## 6. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the performance and reliability of our codes. For any technical issues, please contact me via email Jiarong Lin < ziv.lin.ljr@gmail.com >.

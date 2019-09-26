# LOAM-Livox
## A robust LiDAR Odometry and Mapping (LOAM) package for Livox-LiDAR

<div align="center">
    <img src="https://github.com/hku-mars/loam_livox/blob/master/pics/zym_rotate.gif" width = 45% >
    <img src="https://github.com/hku-mars/loam_livox/blob/master/pics/hkust_stair.gif" width = 45% >
</div>

**Loam-Livox** is a robust, low drift, and real time odometry and mapping package for [*Livox LiDARs*](https://www.livoxtech.com/), significant low cost and high performance LiDARs that are designed for massive industrials uses. Our package address many key issues: feature extraction and selection in a very limited FOV, robust outliers rejection, moving objects filtering, and motion distortion compensation. We use [*Ceres-Solver*](http://ceres-solver.org/) for scan matching to avoid complicated differential geometry derivation. The codes are well structured and streamlined to improve readability and extendability.

In the development of our package, we reference to LOAM, [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED), and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).

**Developer:** [Jiarong Lin](https://github.com/ziv-lin)

**Coming update:** In the following one and half months (before 15/11/2019), we will continuously add the following features to our current project, including **optimized system design**, **parallelable pipeline**, point cloud management using cells and maps, **loop closure**, utilities for maps saving and reload, etc. To know more about the details, please refer to our related paper:)

<div align="center">
    <img src="https://github.com/hku-mars/loam_livox/blob/master/pics/loop_4in1.png" width = 100% >
</div>

**Related paper**: our related paper are now available on arxiv:
1. [Loam_livox: A fast, robust, high-precision LiDAR odometry and mapping package for LiDARs of small FoV](https://arxiv.org/abs/1909.06700)
2. [A fast, complete, point cloud based loop closure for LiDAR odometry and mapping](https://arxiv.org/abs/1909.06700)


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

## 5. Our 3D-printable handheld device
To get our following handheld device, please go to another one of our [open source reposity](https://github.com/ziv-lin/My_solidworks/tree/master/livox_handhold), all of the 3D parts are all designed of FDM printable. We also release our solidwork files so that you can freely make your own adjustments.

<div align="center">
    <img src="https://github.com/hku-mars/loam_livox/blob/master/pics/handheld.png" width=45% >
    <img src="https://github.com/hku-mars/loam_livox/blob/master/pics/system_low_cost.png" width=45% >
</div>

## 6.Acknowledgments
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) LOAM, [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED), and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).

## 7. License
The source code is released under [GPLv2](http://www.gnu.org/licenses/) license.

We are still working on improving the performance and reliability of our codes. For any technical issues, please contact me via email Jiarong Lin < ziv.lin.ljr@gmail.com >.

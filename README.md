# hdl_graph_slam
***hdl_graph_slam*** is an open source ROS package for real-time 3D slam using a 3D LIDAR. It is based on 3D Graph SLAM with NDT scan matching-based odometry estimation and loop detection. It also utilizes floor plane detection to generate an environmental map with a completely flat floor. This package also contains an implementation of multi-threaded NDT (10 times faster than the normal NDT in PCL!). We have tested this packaged mainly in indoor environments, but it can be applied to outdoor environment mapping as well.

<img src="imgs/hdl_graph_slam.png" width="712pix" />

<a href="https://drive.google.com/open?id=0B9f5zFkpn4soSG96Tkt4SFFTbms">video</a>

## Nodelets
***hdl_graph_slam*** consists of four nodelets. 
- *prefiltering_nodelet*
- *scan_matching_odometry_nodelet*
- *floor_detection_nodelet*
- *hdl_graph_slam_nodelet*

The input point cloud is first downsampled by *prefiltering_nodelet*, and then passed to the next nodelets. While *scan_matching_odometry_nodelet* estimates the sensor pose by iteratively applying a scan matching between consecutive frames (i.e., odometry estimation), *floor_detection_nodelet* detects floor planes by RANSAC. The estimated odometry and the detected floor planes are sent to *hdl_graph_slam*. To compensate the accumulated error of the scan matching, it performs loop detection and optimizes a pose graph which takes odometry, loops, and floor planes into account.<br>

**[Sep/1/2017]** A GPS-based graph optimization has been implemented. *hdl_graph_slam_nodelet* receives GPS data (i.e., latitude, longitude) from */gps/geopoint* and converts them into the UTM coordinate, and then it adds them into the pose graph. It effectively compensate the scan matching error in outdoor environments.

<br>
<img src="imgs/nodelets.png" width="712pix" />

### Parameters
All the parameters are listed in *launch/hdl_graph_slam.launch* as ros params.

### Services
- */hdl_graph_slam/dump*  (std_srvs/Empty)
  - save all data (point clouds, floor coeffs, odoms, and pose graph) to the current directory.

## Multi-threaded Scan Matching Algorithms
This package contains implementations of multi-threaded and SSE friendly NDT and GICP algorithms. They are derived from the implementations in PCL. With recent multi-core processors, they may show ten times faster processing speed than the NDT and the GICP in PCL. See *include/pclomp* and *src/pclomp* to use the boosted scan matching algorithms.

## Requirements
***hdl_graph_slam*** requires the following libraries:
- OpenMP
- PCL 1.7
- g2o

Note that ***hdl_graph_slam*** cannot be built with older g2o libraries (such as ros-indigo-libg2o). Install the latest g2o:
```bash
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
make -j8
sudo make install
```

The following ros packages are required:
- geodesy
- pcl_ros
```bash
sudo apt-get install ros-indigo-geodesy ros-indigo-pcl_ros
```

## Example

Example bag files (recorded in a small room): 
- [hdl_501.bag.tar.gz](http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_501.bag.tar.gz) (raw data, 344MB)
- [hdl_501_filtered.bag.tar.gz](http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_501_filtered.bag.tar.gz) (downsampled data, 57MB, **Recommended!**)

```bash
rosparam set use_sim_time true
roslaunch hdl_graph_slam hdl_graph_slam_501.launch
```

```bash
roscd hdl_graph_slam/rviz
rviz -d hdl_graph_slam.rviz
```

```bash
rosbag play --clock hdl_501_filtered.bag
```

We also provide bag_player.py which adjusts the playback speed according to the processing speed of your PC. It allows processing data as fast as possible for your PC.

```bash
rosrun hdl_graph_slam bag_player.py hdl_501_filtered.bag
```

You'll see a generated point cloud like:

<img src="imgs/top.png" height="256pix" /> <img src="imgs/birds.png" height="256pix" /> 


## Example with GPS feature
Ford Campus Vision and Lidar Data Set <a href="http://robots.engin.umich.edu/SoftwareData/Ford">[URL]</a>.

The following script converts the Ford Lidar Dataset to a rosbag and plays it. In this example, ***hdl_graph_slam*** utilized the GPS data to correct the pose graph.

```bash
cd IJRR-Dataset-2
rosrun hdl_graph_slam ford2bag.py dataset-2.bag
rosrun hdl_graph_slam bag_player.py dataset-2.bag
```

<img src="imgs/ford1.png" height="200pix"/> <img src="imgs/ford2.png" height="200pix"/> <img src="imgs/ford3.png" height="200pix"/>

## Papers
Kenji Koide, Jun Miura, and Emanuele Menegatti, A Portable 3D LIDAR-based System for Long-term and Wide-area People Behavior Measurement, Robotics and Autonomous Systems (under review) [PDF].

## Contact
Kenji Koide, Active Intelligent Systems Laboratory, Toyohashi University of Technology <a href="http://www.aisl.cs.tut.ac.jp">[URL]</a> <br>
koide@aisl.cs.tut.ac.jp


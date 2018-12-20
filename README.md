# hdl_graph_slam
***hdl_graph_slam*** is an open source ROS package for real-time 6DOF SLAM using a 3D LIDAR. It is based on 3D Graph SLAM with NDT scan matching-based odometry estimation and loop detection. It also supports several graph constraints, such as GPS, IMU acceleration (gravity vector), IMU orientation (magnetic sensor), and floor plane (detected in a point cloud). We have tested this package with Velodyne (HDL32e, VLP16) and RoboSense (16 channels) sensors in indoor and outdoor environments. 

<img src="imgs/hdl_graph_slam.png" width="712pix" />

<a href="https://drive.google.com/open?id=0B9f5zFkpn4soSG96Tkt4SFFTbms">video</a>


[![Codacy Badge](https://api.codacy.com/project/badge/Grade/1175635f00394e789b457b44690ce72c)](https://app.codacy.com/app/koide3/hdl_graph_slam?utm_source=github.com&utm_medium=referral&utm_content=koide3/hdl_graph_slam&utm_campaign=Badge_Grade_Dashboard)
[![Build Status](https://travis-ci.org/koide3/hdl_graph_slam.svg?branch=master)](https://travis-ci.org/koide3/hdl_graph_slam) on kinetic & melodic

## Nodelets
***hdl_graph_slam*** consists of four nodelets. 
- *prefiltering_nodelet*
- *scan_matching_odometry_nodelet*
- *floor_detection_nodelet*
- *hdl_graph_slam_nodelet*

The input point cloud is first downsampled by *prefiltering_nodelet*, and then passed to the next nodelets. While *scan_matching_odometry_nodelet* estimates the sensor pose by iteratively applying a scan matching between consecutive frames (i.e., odometry estimation), *floor_detection_nodelet* detects floor planes by RANSAC. The estimated odometry and the detected floor planes are sent to *hdl_graph_slam*. To compensate the accumulated error of the scan matching, it performs loop detection and optimizes a pose graph which takes various constraints into account.<br>

<br>
<img src="imgs/nodelets.png" width="712pix" />

## Constraints (Edges)

You can enable/disable each constraint by changing params in the launch file, and you can also change the weight (\*_stddev) and the robust kernel (\*_robust_kernel) of each constraint.


- ***Odometry***

- ***Loop closure***

- ***GPS***
  - */gps/geopoint* (geographic_msgs/GeoPointStamped)
  - */gps/navsat* (sensor_msgs/NavSatFix)
  - */gpsimu_driver/nmea_sentence* (nmea_msgs/Sentence)

hdl_graph_slam supports several GPS message types. All the supported types contain (latitude, longitude, and altitude). hdl_graph_slam converts them into [the UTM coordinate](http://wiki.ros.org/geodesy), and adds them into the graph as 3D position constraints. If altitude is set to NaN, the GPS data is treated as a 2D constrait. GeoPoint is the most basic one, which consists of only (lat, lon, alt). Although NavSatFix provides many information, we use only (lat, lon, alt) and ignore all other data. If you're using HDL32e, you can directly connect *hdl_graph_slam* and *velodyne_driver* via */gpsimu_driver/nmea_sentence*.

- ***IMU acceleration (gravity vector)***
  - */gpsimu_driver/imu_data* (sensor_msgs/Imu)

This constraint rotates each pose node so that the acceleration vector associated with the node will be vertical (as the gravity vector). This is useful to compensate the accumulated tilt rotation error of the scan matching. Since we ignore acceleration by sensor motion, you should not give a big weight for this constraint.

- ***IMU orientation (magnetic sensor)***
  - */gpsimu_driver/imu_data* (sensor_msgs/Imu)

If your IMU has a reliable magnetic orientation sensor, you can add orientation data to the graph as 3D rotation constraints. Note that, magnetic orientation sensors can be affected by external magnetic disturbances. In such cases, this constraint should be disabled.

- ***Floor plane***
  - */floor_detection/floor_coeffs* (hdl_graph_slam/FloorCoeffs)

This constraint optimizes the graph so that the floor planes (detected by RANSAC) of the pose nodes will be the same. This is designed to compensate the accumulated rotation error of the scan matching in large flat indoor environments.


## Parameters
All the configurable parameters are listed in *launch/hdl_graph_slam.launch* as ros params.

## Services
- */hdl_graph_slam/dump*  (hdl_graph_slam/DumpGraph)
  - save all the data (point clouds, floor coeffs, odoms, and pose graph) to a directory.
- */hdl_graph_slam/save_map*  (hdl_graph_slam/SaveMap)
  - save the generated map as a PCD file.

## Requirements
***hdl_graph_slam*** requires the following libraries:
- OpenMP
- PCL 1.7
- g2o
- suitesparse

The following ROS packages are required:
- geodesy
- nmea_msgs
- pcl_ros
- <a href="https://github.com/koide3/ndt_omp">ndt_omp</a>
```bash
# for indigo
sudo apt-get install ros-indigo-geodesy ros-indigo-pcl_ros ros-indigo-nmea-msgs
# for kinetic
sudo apt-get install ros-kinetic-geodesy ros-kinetic-pcl_ros ros-kinetic-nmea-msgs ros-kinetic-libg2o
# for melodic
sudo apt-get install ros-melodic-geodesy ros-melodic-pcl_ros ros-melodic-nmea-msgs ros-melodic-libg2o

cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
```

Note that, in case use are using ros indigo, ***hdl_graph_slam*** cannot be built with the ros g2o binaries (ros-indigo-libg2o). ~~Install the latest g2o:~~
The latest g2o causes segfault. Use commit *a48ff8c42136f18fbe215b02bfeca48fa0c67507* instead of the latest one:

```bash
sudo apt-get install libsuitesparse-dev
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout a48ff8c42136f18fbe215b02bfeca48fa0c67507
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
make -j8
sudo make install
```

**[optional]** *bag_player.py* script requires ProgressBar2.
```bash
sudo pip install ProgressBar2
```

## Example1 (Indoor)

Bag file (recorded in a small room): 
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

We also provide bag_player.py which adjusts the playback speed according to the processing speed of your PC. It allows to process data as fast as possible for your PC.

```bash
rosrun hdl_graph_slam bag_player.py hdl_501_filtered.bag
```

You'll see a point cloud like:

<img src="imgs/top.png" height="256pix" /> <img src="imgs/birds.png" height="256pix" /> 

You can save the generated map by:
```bash
rosservice call /hdl_graph_slam/save_map "resolution: 0.05
destination: '/full_path_directory/map.pcd'"
```

## Example2 (Outdoor)

Bag file (recorded in an outdoor environment): 
- [hdl_400.bag.tar.gz](http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_400.bag.tar.gz) (raw data, about 900MB)


```bash
rosparam set use_sim_time true
roslaunch hdl_graph_slam hdl_graph_slam_400.launch
```

```bash
roscd hdl_graph_slam/rviz
rviz -d hdl_graph_slam.rviz
```

```bash
rosbag play --clock hdl_400.bag
```


<img src="imgs/hdl_400_points.png" height="256pix" /> <img src="imgs/hdl_400_graph.png" height="256pix" /> 


## Example with GPS
Ford Campus Vision and Lidar Data Set <a href="http://robots.engin.umich.edu/SoftwareData/Ford">[URL]</a>.

The following script converts the Ford Lidar Dataset to a rosbag and plays it. In this example, ***hdl_graph_slam*** utilizes the GPS data to correct the pose graph.

```bash
cd IJRR-Dataset-2
rosrun hdl_graph_slam ford2bag.py dataset-2.bag
rosrun hdl_graph_slam bag_player.py dataset-2.bag
```

<img src="imgs/ford1.png" height="200pix"/> <img src="imgs/ford2.png" height="200pix"/> <img src="imgs/ford3.png" height="200pix"/>


## Use hdl_graph_slam in your system

1. Define the transformation between your sensors (LIDAR, IMU, GPS) and the base of your system using static_transform_publisher (see line #11, hdl_graph_slam.launch). All the sensor data will be transformed into the common base frame, and then passed to the SLAM algorithm.

2. Remap the point cloud topic of ***prefiltering_nodelet***. Like: 
  ```bash
  <node pkg="nodelet" type="nodelet" name="prefiltering_nodelet" ...
    <remap from="/velodyne_points" to="/rslidar_points"/>
  ...
  ```

## Common Problems

### hdl_graph_slam_nodelet causes memory error

It has been reported that *hdl_graph_slam_nodelet* causes a memory error in some environments. I found that this is caused by a variable (*color*) in g2o::VertexPlane. Since this field is used for only visualization, we can remove it from vertex_plane.h and vertex_plane.cpp in g2o. I made a clone repository of g2o, in which I just removed it
 from the commit *a48ff8c42136f18fbe215b02bfeca48fa0c67507* of g2o. If you face this memory error problem, try to install it instead of the original g2o repository. Do not forget to checkout *hdl_graph_slam* branch.

```bash
git clone https://github.com/koide3/g2o.git
cd g2o
git checkout hdl_graph_slam
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
make -j8
sudo make install
```

### hdl_graph_slam in docker

If you still have the error, try our docker environment. You can build the docker image for *hdl_graph_slam* with: 

```bash
roscd hdl_graph_slam
sudo docker build --tag hdl_graph_slam -f docker/kinetic/Dockerfile .
# you can also use melodic environment
# sudo docker build --tag hdl_graph_slam -f docker/melodic/Dockerfile .
```

After building the image, you can launch hdl_graph_slam with:

```bash
sudo docker run -it --net=host --rm hdl_graph_slam bash
source /root/catkin_ws/devel/setup.bash
roslaunch hdl_graph_slam hdl_graph_slam.launch
```

## Related packages

- <a href="https://github.com/koide3/hdl_graph_slam">hdl_graph_slam</a>
- <a href="https://github.com/koide3/hdl_localization">hdl_localization</a>
- <a href="https://github.com/koide3/hdl_people_tracking">hdl_people_tracking</a>

<img src="imgs/packages.png"/>


## Papers
Kenji Koide, Jun Miura, and Emanuele Menegatti, A Portable 3D LIDAR-based System for Long-term and Wide-area People Behavior Measurement, IEEE Transactions on Human-Machine Systems (under review) [PDF].

## Contact
Kenji Koide, Active Intelligent Systems Laboratory, Toyohashi University of Technology <a href="http://www.aisl.cs.tut.ac.jp">[URL]</a> <br>
koide@aisl.cs.tut.ac.jp


# hdl_graph_slam

Original repository: https://github.com/koide3/hdl_graph_slam


## Build
```bash
cd hdl_graph_slam/docker
./build.sh
```

## Run

### On host:
```bash
roscore
```

```bash
rosparam set use_sim_time true

cd hdl_graph_slam/rviz
rviz -d hdl_graph_slam.rviz
```

```bash
rosbag play --clock hdl_400.bag
```
http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_400.bag.tar.gz

### On docker image:
```bash
cd hdl_graph_slam/docker
./run.sh

roslaunch hdl_graph_slam hdl_graph_slam_400.launch
```


![hdl_graph_slam](https://user-images.githubusercontent.com/31344317/98347836-4fed5a00-205b-11eb-931c-158f6cd056bf.gif)

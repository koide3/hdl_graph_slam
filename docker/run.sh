#!/bin/bash
docker run --net=host -it --rm \
           -v $(realpath ..):/root/catkin_ws/src/hdl_graph_slam \
           -w /root/catkin_ws/src/hdl_graph_slam \
           $@ \
           hdl_graph_slam

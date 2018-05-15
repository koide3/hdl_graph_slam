FROM ros:kinetic

RUN apt-get update
RUN apt-get install -y
RUN apt-get install -y wget nano build-essential
RUN apt-get install -y ros-kinetic-geodesy ros-kinetic-pcl-ros ros-kinetic-nmea-msgs


WORKDIR /root
RUN git clone https://github.com/koide3/g2o.git
WORKDIR /root/g2o
RUN git checkout hdl_graph_slam
WORKDIR /root/g2o/build
RUN mkdir build
RUN cmake .. -DCMAKE_BUILD_TYPE=RELEASE
RUN make -j$(grep -c ^processor /proc/cpuinfo)
RUN make install

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace'
RUN git clone https://github.com/koide3/ndt_omp.git
RUN git clone https://github.com/koide3/hdl_graph_slam.git

RUN apt-get install -y python-pip
RUN apt-get install -y ros-kinetic-rviz 
RUN apt-get install -y ros-kinetic-tf-conversions

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_make'
RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

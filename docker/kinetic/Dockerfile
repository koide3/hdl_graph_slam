FROM ros:kinetic

RUN apt-get update && apt-get install -y && apt-get install -y wget nano build-essential ros-kinetic-geodesy ros-kinetic-pcl-ros ros-kinetic-nmea-msgs ros-kinetic-rviz ros-kinetic-tf-conversions ros-kinetic-libg2o

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace'
RUN git clone https://github.com/koide3/ndt_omp.git
# RUN git clone https://github.com/koide3/hdl_graph_slam.git
ADD . /root/catkin_ws/src/hdl_graph_slam/

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_make'
RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

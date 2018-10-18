FROM ros:kinetic

RUN apt-get update
RUN apt-get update
RUN apt-get install -y
RUN apt-get install -y wget nano build-essential
RUN apt-get install -y python-pip
RUN apt-get install -y ros-kinetic-geodesy ros-kinetic-pcl-ros ros-kinetic-nmea-msgs ros-kinetic-rviz ros-kinetic-tf-conversions
RUN apt-get install -y ros-kinetic-libg2o

RUN echo te
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace'
RUN git clone https://github.com/koide3/ndt_omp.git
RUN git clone https://github.com/koide3/hdl_graph_slam.git

WORKDIR /root/catkin_ws
# RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_make'
# RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

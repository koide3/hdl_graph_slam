FROM ros:melodic

RUN apt-get update && apt-get install -y
RUN apt-get update && apt-get install -y wget nano build-essential
RUN apt-get update && apt-get install -y ros-melodic-geodesy ros-melodic-pcl-ros ros-melodic-nmea-msgs ros-melodic-rviz ros-melodic-tf-conversions
RUN apt-get update && apt-get install -y ros-melodic-libg2o

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace'
RUN git clone https://github.com/koide3/ndt_omp.git
RUN git clone https://github.com/koide3/hdl_graph_slam.git && cd hdl_graph_slam && git checkout arch-melodic

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'
RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

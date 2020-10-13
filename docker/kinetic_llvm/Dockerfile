FROM ros:kinetic

RUN apt-get update && apt-get install --no-install-recommends -y \
 && apt-get install --no-install-recommends -y \
 wget nano build-essential libomp-dev clang lld-6.0 \
 ros-kinetic-geodesy ros-kinetic-pcl-ros ros-kinetic-nmea-msgs \
 ros-kinetic-rviz ros-kinetic-tf-conversions ros-kinetic-libg2o \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/ld ld /usr/bin/ld.lld-6.0 50

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace'
RUN git clone https://github.com/koide3/ndt_omp.git
RUN git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive

COPY . /root/catkin_ws/src/hdl_graph_slam/

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; CC=clang CXX=clang++ catkin_make'
RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

#!/bin/bash

sudo docker run -it --net=host --env=DISPLAY --env=QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --rm -p 52698:52698 hdl_graph_slam bash

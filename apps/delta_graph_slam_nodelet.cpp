#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace hdl_graph_slam {

class DeltaGraphSlamNodelet : public nodelet::Nodelet {
    virtual void onInit() {
        ROS_INFO("First nodelet created");
    }
};
}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::DeltaGraphSlamNodelet, nodelet::Nodelet)
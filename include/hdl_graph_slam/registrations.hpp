#ifndef HDL_GRAPH_SLAM_REGISTRATIONS_HPP
#define HDL_GRAPH_SLAM_REGISTRATIONS_HPP

#include <ros/ros.h>

#include <pcl/registration/registration.h>

namespace hdl_graph_slam {

/**
 * @brief select a scan matching algorithm according to rosparams
 * @param pnh
 * @return selected scan matching
 */
boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> select_registration_method(ros::NodeHandle& pnh);

}

#endif //

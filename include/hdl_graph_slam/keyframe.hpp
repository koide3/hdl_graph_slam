#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>

namespace g2o {
  class VertexSE3;
}

namespace hdl_graph_slam {

/**
 * @brief KeyFrame (pose node)
 */
struct KeyFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PointT = pcl::PointXYZI;
  using Ptr = std::shared_ptr<KeyFrame>;

  KeyFrame(const ros::Time& stamp, const Eigen::Isometry3d& odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr& cloud);
  ~KeyFrame();

  void dump(const std::string& directory);

public:
  ros::Time stamp;                                // timestamp
  Eigen::Isometry3d odom;                         // odometry (estimated by scan_matching_odometry)
  double accum_distance;                          // accumulated distance from the first node (by scan_matching_odometry)
  pcl::PointCloud<PointT>::ConstPtr cloud;        // point cloud
  boost::optional<Eigen::Vector4d> floor_coeffs;  // detected floor's coefficients
  boost::optional<Eigen::Vector3d> utm_coord;     // UTM coord obtained by GPS

  g2o::VertexSE3* node;                           // node instance
};

/**
 * @brief KeyFramesnapshot for map cloud generation
 */
struct KeyFrameSnapshot {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PointT = KeyFrame::PointT;
  using Ptr = std::shared_ptr<KeyFrameSnapshot>;

  KeyFrameSnapshot(const KeyFrame::Ptr& key);
  KeyFrameSnapshot(const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud);

  ~KeyFrameSnapshot();

public:
  Eigen::Isometry3d pose;                   // pose estimated by graph optimization
  pcl::PointCloud<PointT>::ConstPtr cloud;  // point cloud
};

}

#endif // KEYFRAME_HPP

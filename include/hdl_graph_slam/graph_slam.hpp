#ifndef GRAPH_SLAM_HPP
#define GRAPH_SLAM_HPP

#include <memory>
#include <ros/time.h>

#include <g2o/core/sparse_optimizer.h>

namespace g2o {
  class VertexSE3;
  class VertexPlane;
  class VertexPointXYZ;
  class EdgeSE3;
  class EdgeSE3Plane;
  class EdgeSE3PointXYZ;
  class EdgeSE3PriorXY;
  class EdgeSE3PriorXYZ;
}

namespace hdl_graph_slam {

class GraphSLAM {
public:
  GraphSLAM();
  ~GraphSLAM();

  /**
   * @brief add a SE3 node to the graph
   * @param pose
   * @return registered node
   */
  g2o::VertexSE3* add_se3_node(const Eigen::Isometry3d& pose);

  /**
   * @brief add a plane node to the graph
   * @param plane_coeffs
   * @return registered node
   */
  g2o::VertexPlane* add_plane_node(const Eigen::Vector4d& plane_coeffs);

  /**
   * @brief add a point_xyz node to the graph
   * @param xyz
   * @return registered node
   */
  g2o::VertexPointXYZ* add_point_xyz_node(const Eigen::Vector3d& xyz);

  /**
   * @brief add an edge between SE3 nodes
   * @param v1  node1
   * @param v2  node2
   * @param relative_pose  relative pose between node1 and node2
   * @param information_matrix  information matrix (it must be 6x6)
   * @return registered edge
   */
  g2o::EdgeSE3* add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief add an edge between an SE3 node and a plane node
   * @param v_se3    SE3 node
   * @param v_plane  plane node
   * @param plane_coeffs  plane coefficients w.r.t. v_se3
   * @param information_matrix  information matrix (it must be 3x3)
   * @return registered edge
   */
  g2o::EdgeSE3Plane* add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief add an edge between an SE3 node and a point_xyz node
   * @param v_se3        SE3 node
   * @param v_xyz        point_xyz node
   * @param xyz          xyz coordinate
   * @param information  information_matrix (it must be 3x3)
   * @return registered edge
   */
  g2o::EdgeSE3PointXYZ* add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief add a prior edge to an SE3 node
   * @param v_se3
   * @param xy
   * @param information_matrix
   * @return
   */
  g2o::EdgeSE3PriorXY* add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix);

  g2o::EdgeSE3PriorXYZ* add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief perform graph optimization
   */
  void optimize();

  /**
   * @brief save the pose graph
   * @param filename  output filename
   */
  void save(const std::string& filename);

public:
  std::unique_ptr<g2o::SparseOptimizer> graph;  // g2o graph
  g2o::VertexPlane* floor_plane_node;           // ground floor plane node
};

}

#endif // GRAPH_SLAM_HPP

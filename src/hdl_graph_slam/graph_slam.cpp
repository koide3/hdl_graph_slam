#include <hdl_graph_slam/graph_slam.hpp>

#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>

G2O_USE_OPTIMIZATION_LIBRARY(csparse)

namespace g2o {
  G2O_REGISTER_TYPE(EDGE_SE3_PLANE, EdgeSE3Plane)
  G2O_REGISTER_TYPE(EDGE_SE3_PRIORXY, EdgeSE3PriorXY)
  G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
}

namespace hdl_graph_slam {

/**
 * @brief constructor
 */
GraphSLAM::GraphSLAM() {
  graph.reset(new g2o::SparseOptimizer());

  std::string g2o_solver_name = "lm_var";
  std::cout << "construct solver... " << std::endl;
  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver = solver_factory->construct(g2o_solver_name, solver_property);
  graph->setAlgorithm(solver);

  if (!graph->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate solver!!" << std::endl;
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }
  std::cout << "done" << std::endl;

  floor_plane_node = add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
  floor_plane_node->setFixed(true);
}

/**
 * @brief destructor
 */
GraphSLAM::~GraphSLAM() {
  graph.reset();
}


g2o::VertexSE3* GraphSLAM::add_se3_node(const Eigen::Isometry3d& pose) {
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(graph->vertices().size());
  vertex->setEstimate(pose);
  graph->addVertex(vertex);

  return vertex;
}

g2o::VertexPlane* GraphSLAM::add_plane_node(const Eigen::Vector4d& plane_coeffs) {
  g2o::VertexPlane* vertex(new g2o::VertexPlane());
  vertex->setId(graph->vertices().size());
  vertex->setEstimate(plane_coeffs);
  graph->addVertex(vertex);

  return vertex;
}

g2o::VertexPointXYZ* GraphSLAM::add_point_xyz_node(const Eigen::Vector3d& xyz) {
  g2o::VertexPointXYZ* vertex(new g2o::VertexPointXYZ());
  vertex->setId(graph->vertices().size());
  vertex->setEstimate(xyz);
  graph->addVertex(vertex);

  return vertex;
}

g2o::EdgeSE3* GraphSLAM::add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3* edge(new g2o::EdgeSE3());
  edge->setMeasurement(relative_pose);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3Plane* GraphSLAM::add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3Plane* edge(new g2o::EdgeSE3Plane());
  edge->setMeasurement(plane_coeffs);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_plane;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PointXYZ* GraphSLAM::add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PointXYZ* edge(new g2o::EdgeSE3PointXYZ());
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_xyz;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorXY* GraphSLAM::add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorXY* edge(new g2o::EdgeSE3PriorXY());
  edge->setMeasurement(xy);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorXYZ* GraphSLAM::add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}


void GraphSLAM::optimize() {
  if(graph->edges().size() < 10) {
    return;
  }

  std::cout << std::endl;
  std::cout << "--- pose graph optimization ---" << std::endl;
  std::cout << "nodes: " << graph->vertices().size() << "   edges: " << graph->edges().size() << std::endl;
  std::cout << "optimizing... " << std::flush;

  graph->initializeOptimization();
  graph->setVerbose(false);

  double chi2 = graph->chi2();

  auto t1 = ros::Time::now();
  int iterations = graph->optimize(1024);

  auto t2 = ros::Time::now();
  std::cout << "done" << std::endl;
  std::cout << "iterations: " << iterations << std::endl;
  std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
  std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;
}

void GraphSLAM::save(const std::string& filename) {
  std::ofstream ofs(filename);
  graph->save(ofs);
}

}

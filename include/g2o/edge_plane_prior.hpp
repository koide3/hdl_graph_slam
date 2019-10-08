#ifndef EDGE_PLANE_PRIOR_HPP
#define EDGE_PLANE_PRIOR_HPP

#include <Eigen/Dense>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

namespace g2o {
class EdgePlanePriorNormal : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexPlane> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePlanePriorNormal() : g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexPlane>() {}

  void computeError() override {
    const g2o::VertexPlane* v1 = static_cast<const g2o::VertexPlane*>(_vertices[0]);
    Eigen::Vector3d normal = v1->estimate().normal();

    if(normal.dot(_measurement) < 0.0) {
      normal = -normal;
    }

    _error = normal - _measurement;
  }

  void setMeasurement(const Eigen::Vector3d& m) override {
    _measurement = m;
  }

  virtual bool read(std::istream& is) override {
    Eigen::Vector3d v;
    is >> v(0) >> v(1) >> v(2);
    setMeasurement(v);
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if(i != j) information()(j, i) = information()(i, j);
      }
    return true;
  }
  virtual bool write(std::ostream& os) const override {
    Eigen::Vector3d v = _measurement;
    os << v(0) << " " << v(1) << " " << v(2) << " ";
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
    return os.good();
  }
};

class EdgePlanePriorDistance : public g2o::BaseUnaryEdge<1, double, g2o::VertexPlane> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePlanePriorDistance() : g2o::BaseUnaryEdge<1, double, g2o::VertexPlane>() {}

  void computeError() override {
    const g2o::VertexPlane* v1 = static_cast<const g2o::VertexPlane*>(_vertices[0]);
    _error[0] = _measurement - v1->estimate().distance();
  }

  void setMeasurement(const double& m) override {
    _measurement = m;
  }

  virtual bool read(std::istream& is) override {
    is >> _measurement;
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if(i != j) information()(j, i) = information()(i, j);
      }
    return true;
  }
  virtual bool write(std::ostream& os) const override {
    os << _measurement;
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
    return os.good();
  }
};
}  // namespace g2o

#endif  // EDGE_SE3_PRIORXY_HPP

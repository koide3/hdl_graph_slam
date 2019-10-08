#ifndef EDGE_PLANE_PARALLEL_HPP
#define EDGE_PLANE_PARALLEL_HPP

#include <Eigen/Dense>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

namespace g2o {

class EdgePlaneParallel : public BaseBinaryEdge<3, Eigen::Vector3d, VertexPlane, VertexPlane> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePlaneParallel() : BaseBinaryEdge<3, Eigen::Vector3d, VertexPlane, VertexPlane>() {
    _information.setIdentity();
    _error.setZero();
  }

  void computeError() override {
    const VertexPlane* v1 = static_cast<const VertexPlane*>(_vertices[0]);
    const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);

    Eigen::Vector3d normal1 = v1->estimate().normal();
    Eigen::Vector3d normal2 = v2->estimate().normal();

    if (normal1.dot(normal2) < 0.0) {
      normal2 = -normal2;
    }

    _error = (normal2 - normal1) - _measurement;
  }
  virtual bool read(std::istream& is) override {
    Eigen::Vector3d v;
    for (int i = 0; i < 3; ++i) {
      is >> v[i];
    }

    setMeasurement(v);
    for(int i = 0; i < information().rows(); ++i) {
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if(i != j) {
          information()(j, i) = information()(i, j);
        }
      }
    }

    return true;
  }

  virtual bool write(std::ostream& os) const override {
    for(int i = 0; i < 3; ++i) {
      os << _measurement[i] << " ";
    }

    for(int i = 0; i < information().rows(); ++i) {
      for(int j = i; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      };
    }
    return os.good();
  }

  virtual void setMeasurement(const Eigen::Vector3d& m) override { _measurement = m; }

  virtual int measurementDimension() const override { return 3; }
};

class EdgePlanePerpendicular : public BaseBinaryEdge<1, Eigen::Vector3d, VertexPlane, VertexPlane> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePlanePerpendicular() : BaseBinaryEdge<1, Eigen::Vector3d, VertexPlane, VertexPlane>() {
    _information.setIdentity();
    _error.setZero();
  }

  void computeError() override {
    const VertexPlane* v1 = static_cast<const VertexPlane*>(_vertices[0]);
    const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);

    Eigen::Vector3d normal1 = v1->estimate().normal().normalized();
    Eigen::Vector3d normal2 = v2->estimate().normal().normalized();

    _error[0] = normal1.dot(normal2);
  }
  virtual bool read(std::istream& is) override {
    Eigen::Vector3d v;
    for (int i = 0; i < 3; ++i) {
      is >> v[i];
    }

    setMeasurement(v);
    for(int i = 0; i < information().rows(); ++i) {
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if (i != j) {
          information()(j, i) = information()(i, j);
        }
      }
    }

    return true;
  }

  virtual bool write(std::ostream& os) const override {
    for (int i = 0; i < 3; ++i) {
      os << _measurement[i] << " ";
    }

    for(int i = 0; i < information().rows(); ++i) {
      for(int j = i; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      };
    }
    return os.good();
  }

  virtual void setMeasurement(const Eigen::Vector3d& m) override { _measurement = m; }

  virtual int measurementDimension() const override { return 3; }
};

}  // namespace g2o

#endif  // EDGE_PLANE_PARALLEL_HPP

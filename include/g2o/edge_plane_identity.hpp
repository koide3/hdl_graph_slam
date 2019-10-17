#ifndef EDGE_PLANE_IDENTITY_HPP
#define EDGE_PLANE_IDENTITY_HPP

#include <Eigen/Dense>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

namespace g2o {

/**
 * @brief A modified version of g2o::EdgePlane. This class takes care of flipped plane normals.
 *
 */
class EdgePlaneIdentity : public BaseBinaryEdge<4, Eigen::Vector4d, VertexPlane, VertexPlane> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePlaneIdentity() : BaseBinaryEdge<4, Eigen::Vector4d, VertexPlane, VertexPlane>() {
    _information.setIdentity();
    _error.setZero();
  }
  void computeError() {
    const VertexPlane* v1 = static_cast<const VertexPlane*>(_vertices[0]);
    const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);

    Eigen::Vector4d p1 = v1->estimate().toVector();
    Eigen::Vector4d p2 = v2->estimate().toVector();

    if(p1.dot(p2) < 0.0) {
      p2 = -p2;
    }

    _error = (p2 - p1) - _measurement;
  }
  virtual bool read(std::istream& is) override {
    Eigen::Vector4d v;
    for(int i = 0; i < 4; ++i) {
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
    for(int i = 0; i < 4; ++i) {
      os << _measurement[i] << " ";
    }

    for(int i = 0; i < information().rows(); ++i) {
      for(int j = i; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      };
    }
    return os.good();
  }

  virtual void setMeasurement(const Eigen::Vector4d& m) override {
    _measurement = m;
  }

  virtual int measurementDimension() const override {
    return 4;
  }
};

}  // namespace g2o

#endif  // EDGE_PLANE_PARALLEL_HPP

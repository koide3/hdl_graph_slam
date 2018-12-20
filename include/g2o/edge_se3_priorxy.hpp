#ifndef EDGE_SE3_PRIORXY_HPP
#define EDGE_SE3_PRIORXY_HPP

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {
  class EdgeSE3PriorXY : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3PriorXY()
      : g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3>()
    {}

    void computeError() override {
      const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

      Eigen::Vector2d estimate = v1->estimate().translation().head<2>();
      _error = estimate - _measurement;
    }

    void setMeasurement(const Eigen::Vector2d& m) override {
      _measurement = m;
    }

    virtual bool read(std::istream& is) override {
      Eigen::Vector2d v;
      is >> v(0) >> v(1);
      setMeasurement(v);
      for (int i = 0; i < information().rows(); ++i)
        for (int j = i; j < information().cols(); ++j) {
          is >> information()(i, j);
          if (i != j)
            information()(j, i) = information()(i, j);
        }
      return true;
    }
    virtual bool write(std::ostream& os) const override {
      Eigen::Vector2d v = _measurement;
      os << v(0) << " " << v(1) << " ";
      for (int i = 0; i < information().rows(); ++i)
        for (int j = i; j < information().cols(); ++j)
          os << " " << information()(i, j);
      return os.good();
    }
  };
}

#endif // EDGE_SE3_PRIORXY_HPP

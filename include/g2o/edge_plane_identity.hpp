// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

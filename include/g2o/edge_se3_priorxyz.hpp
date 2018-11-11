#ifndef KKL_G2O_EDGE_SE3_PRIORXYZ_HPP
#define KKL_G2O_EDGE_SE3_PRIORXYZ_HPP

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {
  class EdgeSE3PriorXYZ : public g2o::BaseUnaryEdge<3, g2o::Vector3, g2o::VertexSE3> {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		EdgeSE3PriorXYZ()
      : g2o::BaseUnaryEdge<3, g2o::Vector3, g2o::VertexSE3>()
		{}

		void computeError() override {
			const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

            g2o::Vector3 estimate = v1->estimate().translation();
			_error = estimate - _measurement;
		}

    void setMeasurement(const g2o::Vector3& m) override {
			_measurement = m;
		}

		virtual bool read(std::istream& is) override {
      g2o::Vector3 v;
			is >> v(0) >> v(1) >> v(2);
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
      g2o::Vector3 v = _measurement;
			os << v(0) << " " << v(1) << " " << v(2) << " ";
			for (int i = 0; i < information().rows(); ++i)
				for (int j = i; j < information().cols(); ++j)
					os << " " << information()(i, j);
			return os.good();
		}
	};
}

#endif

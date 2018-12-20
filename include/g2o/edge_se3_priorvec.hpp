#ifndef KKL_G2O_EDGE_SE3_PRIORVEC_HPP
#define KKL_G2O_EDGE_SE3_PRIORVEC_HPP

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {
class EdgeSE3PriorVec : public g2o::BaseUnaryEdge<3, Eigen::Matrix<double, 6, 1>, g2o::VertexSE3> {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		EdgeSE3PriorVec()
		: g2o::BaseUnaryEdge<3, Eigen::Matrix<double, 6, 1>, g2o::VertexSE3>()
		{}

		void computeError() override {
			const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

			Eigen::Vector3d direction = _measurement.head<3>();
			Eigen::Vector3d measurement = _measurement.tail<3>();

			Eigen::Vector3d estimate = (v1->estimate().linear().inverse() * direction);

			_error = estimate - measurement;
		}

		void setMeasurement(const Eigen::Matrix<double, 6, 1>& m) override {
			_measurement.head<3>() = m.head<3>().normalized();
			_measurement.tail<3>() = m.tail<3>().normalized();
		}

		virtual bool read(std::istream& is) override {
			Eigen::Matrix<double, 6, 1> v;
			is >> v[0] >> v[1] >> v[2] >> v[3] >> v[4] >> v[5];
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
			Eigen::Matrix<double, 6, 1> v = _measurement;
			os << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << " " << v[4] << " " << v[5];
			for (int i = 0; i < information().rows(); ++i)
				for (int j = i; j < information().cols(); ++j)
					os << " " << information()(i, j);
			return os.good();
		}
	};
}

#endif

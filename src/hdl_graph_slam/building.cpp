#include <hdl_graph_slam/building.hpp>

namespace hdl_graph_slam {
	
Building::Building(void) {geometry = pcl::PointCloud<PointT3>::Ptr(new pcl::PointCloud<PointT3>);}

}
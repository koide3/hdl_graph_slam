#include "hdl_graph_slam/building_node.hpp"

namespace hdl_graph_slam {

BuildingNode::BuildingNode() { referenceSystem = pcl::PointCloud<PointT3>::Ptr(new pcl::PointCloud<PointT3>); node = nullptr; }

// center all points around local origin
void BuildingNode::setReferenceSystem() {
	setOrigin();
	pcl::PointCloud<PointT3>::Ptr geometry = building.geometry;
 
	for(int i = 0; i< geometry->size(); i++) {
		PointT3 pt_temp = geometry->at(i);
		pt_temp.x = pt_temp.x - local_origin(0);
		pt_temp.y = pt_temp.y - local_origin(1);
		pt_temp.z = 0;
		referenceSystem->push_back(pt_temp);
	}
	return;
}

// use first point as local_origin
void BuildingNode::setOrigin() {
	PointT3 first_point = building.geometry->at(0);

	local_origin(0) = first_point.x;
	local_origin(1) = first_point.y;

	return;
}

}
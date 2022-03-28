#include "hdl_graph_slam/building_node.hpp"

namespace hdl_graph_slam {

BuildingNode::BuildingNode() { referenceSystem = pcl::PointCloud<PointT3>::Ptr(new pcl::PointCloud<PointT3>); node = nullptr; }

// this method refers all points of the building pc to local_origin
void BuildingNode::setReferenceSystem() {
	setOrigin(); // set local_origin
	pcl::PointCloud<PointT3>::Ptr geometry = building.geometry;

	// for to correct all points as pt = pt - local_origin 
	for(int i = 0; i< geometry->size(); i++) {
		PointT3 pt_temp = geometry->at(i);
		pt_temp.x = pt_temp.x - local_origin(0);
		pt_temp.y = pt_temp.y - local_origin(1);
		pt_temp.z = 0;
		referenceSystem->push_back(pt_temp);
	}
	return;		
}

// set local_origin = south-westernmost point of the building
void BuildingNode::setOrigin() {
	pcl::PointCloud<PointT3>::Ptr geometry = building.geometry;
	int sw = 1;
	PointT3 maxp;

	// the south-westernmost point is the one with lowest x and y
	for(int i = 0; i< geometry->size(); i++) {
		PointT3 pt_temp = geometry->at(i);
		if(sw == 1) {
			maxp = pt_temp;
			sw = 0;
		} else {
			if(pt_temp.x < maxp.x) {
				maxp = pt_temp;
			}
		}
	}
	// convert maxp into Eigen::Vector3d
	Eigen::Vector2d result;
	result(0) = maxp.x;
	result(1) = maxp.y;
	//std::cout << "local origin: " << result(0) << ", " << result(1) << std::endl;
	local_origin = result;
	return;
}

}
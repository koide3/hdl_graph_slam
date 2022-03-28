#ifndef BUILDING_NODE
#define BUILDING_NODE

#include <ros/ros.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <hdl_graph_slam/building.hpp>

namespace g2o {
  class VertexSE2;
}

namespace hdl_graph_slam {
	
class BuildingNode {

private:
	void setOrigin();						
public: 
	typedef boost::shared_ptr<BuildingNode> Ptr; 
	Building building;							// building to which the node refers
	pcl::PointCloud<PointT3>::Ptr referenceSystem;	// pc containing all building points referred to local_origin (from building.geometry) 
	Eigen::Vector2d local_origin; 				// south-westernmost point of the building wrt zero_utm
	g2o::VertexSE2* node;

	BuildingNode();
	void setReferenceSystem();
	pcl::PointCloud<PointT3>::Ptr setVerticesReferenceSystem();
};

}
#endif
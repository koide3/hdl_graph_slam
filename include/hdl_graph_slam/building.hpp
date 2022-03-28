#ifndef BUILDING_HPP
#define BUILDING_HPP

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

namespace hdl_graph_slam {

using PointT3 = pcl::PointXYZ;

class Building {
	public: 
		std::string id;
		std::map<std::string,std::string> tags;
		pcl::PointCloud<PointT3>::Ptr geometry; // already interpolated and referred to zero utm
		pcl::PointCloud<PointT3>::Ptr vertices;
		Building(void);
};

}
#endif
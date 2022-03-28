#ifndef BUILDING_TOOLS
#define BUILDING_TOOLS

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <hdl_graph_slam/building.hpp>
#include <fstream>
#include <regex>
#include <string>
#include <cmath>
#include <sstream>
#include <cstdlib>
#include <iomanip>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Exception.hpp>

namespace hdl_graph_slam {

class BuildingTools {
public: 
	static std::vector<Building> getBuildings(double lat, double lon, double rad, Eigen::Vector2d zero_utm, std::string host);
	struct Node
	{
		std::string id;
		double lat;
		double lon;
	};
	BuildingTools(void);
private:
	
	static std::string downloadBuildings(double lat, double lon, double rad, std::string host);
	static std::vector<Building> parseBuildings(std::string result, Eigen::Vector3d zero_utm);
	static pcl::PointCloud<PointT3>::Ptr buildPointCloud(std::vector<std::string> nd_refs, std::vector<Node> nodes,Eigen::Vector3d zero_utm);
	static pcl::PointCloud<PointT3>::Ptr getVertices(std::vector<std::string> nd_refs, std::vector<Node> nodes,Eigen::Vector3d zero_utm);
	static Node getNode(std::string nd_ref, std::vector<Node> nodes);
	static PointT3 toUtm(Eigen::Vector3d pt, Eigen::Vector3d zero_utm);
	static pcl::PointCloud<PointT3>::Ptr interpolate(PointT3 a, PointT3 b);	
};

}
#endif
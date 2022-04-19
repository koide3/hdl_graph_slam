#ifndef BUILDING_TOOLS
#define BUILDING_TOOLS

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <hdl_graph_slam/building.hpp>
#include <pcl/common/distances.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <regex>
#include <string>
#include <cmath>
#include <sstream>
#include <cstdlib>
#include <iomanip>
#include <map>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Exception.hpp>
namespace pt = boost::property_tree;

namespace hdl_graph_slam {

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

class BuildingTools {
public:
	typedef boost::shared_ptr<BuildingTools> Ptr;
	BuildingTools() {}
	BuildingTools(std::string host, Eigen::Vector2d zero_utm, double radius=40, double buffer_radius=1000): host(host), zero_utm(zero_utm), radius(radius), buffer_radius(buffer_radius) {}
	std::vector<Building::Ptr> getBuildings(double lat, double lon);
private:
	struct Node {
		std::string id;
		double lat;
		double lon;
	};
	std::string host;
	Eigen::Vector2d zero_utm;
	double radius;
	double buffer_radius;
	Eigen::Vector3f buffer_center;
	std::vector<Node> nodes;
	pt::ptree xml_tree;
	std::map<std::string,Building::Ptr> buildings_map;
	std::vector<Building::Ptr> buildings;
	void downloadBuildings(double lat, double lon);
	void parseBuildings(double lat, double lon);
	pcl::PointCloud<PointT3>::Ptr getVertices(std::vector<std::string> nd_refs);
	pcl::PointCloud<PointT3>::Ptr buildPointCloud(std::vector<std::string> nd_refs);
	Node getNode(std::string nd_ref);
	pcl::PointCloud<PointT3>::Ptr interpolate(PointT3 a, PointT3 b);	
	PointT3 toEnu(Eigen::Vector3d lla);
	bool isBuildingInRadius(pt::ptree::value_type &tree_node, double lat, double lon);
};

}
#endif
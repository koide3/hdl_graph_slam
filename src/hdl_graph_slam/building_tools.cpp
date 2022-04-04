#include "hdl_graph_slam/building_tools.hpp"

//DEBUG = 1 -> print debug lines
//DEBUG different from 1 -> print nothing
#define DEBUG 0

namespace hdl_graph_slam {

BuildingTools::BuildingTools(void) {
}

std::vector<Building> BuildingTools::parseBuildings(std::string result, Eigen::Vector3d zero_utm) {
	std::stringstream ss(result);
	boost::property_tree::ptree pt;
	read_xml(ss, pt);
	std::vector< Node> nodes;
	std::vector<Building> b;
	try {
		BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, pt.get_child("osm") ) {
			if( v.first == "node" ) {
					Node n;
				n.id = v.second.get<std::string>("<xmlattr>.id");
				n.lat = v.second.get<double>("<xmlattr>.lat");
				n.lon = v.second.get<double>("<xmlattr>.lon");
				nodes.push_back(n);
			}
			if(v.first == "way") {
				Building btemp;
				std::vector<std::string> nd_refs;
				std::map<std::string,std::string> tags;
				std::string way_id = v.second.get<std::string>("<xmlattr>.id");
				btemp.id = way_id;

				for(boost::property_tree::ptree::const_iterator v1 = v.second.begin(); v1 != v.second.end(); ++v1) {
					if(v1->first == "nd") {
						std::string nd_ref = v1->second.get<std::string>("<xmlattr>.ref");
						nd_refs.push_back(nd_ref);
					}
					if(v1->first == "tag") {
						std::string key = v1->second.get<std::string>("<xmlattr>.k");
						std::string value = v1->second.get<std::string>("<xmlattr>.v");
						tags[key] = value;
					}
				}
				btemp.tags = tags;
				pcl::PointCloud<PointT3>::Ptr pc_temp(new pcl::PointCloud<PointT3>);
				*pc_temp = *(buildPointCloud(nd_refs,nodes,zero_utm));
				btemp.geometry = pc_temp;
				btemp.vertices = getVertices(nd_refs,nodes,zero_utm);
				b.push_back(btemp);
			}
		}
	}
	catch(boost::property_tree::ptree_error & e){
		std::cerr<< "No xml! error:" << e.what() << std::endl;
	}
	return b;
}

pcl::PointCloud<PointT3>::Ptr BuildingTools::getVertices(std::vector<std::string> nd_refs, std::vector< Node> nodes, Eigen::Vector3d zero_utm) {
	pcl::PointCloud<PointT3>::Ptr pc_temp(new pcl::PointCloud<PointT3>);
	for(std::vector<std::string>::const_iterator it = nd_refs.begin(); it != nd_refs.end(); ++it) {
		Node n = getNode(*it, nodes);
		PointT3 pt_temp = toUtm(Eigen::Vector3d(n.lat, n.lon, 0), zero_utm);
		pc_temp->push_back(pt_temp);
	}
	return pc_temp;		
}

pcl::PointCloud<PointT3>::Ptr BuildingTools::buildPointCloud(std::vector<std::string> nd_refs, std::vector< Node> nodes, Eigen::Vector3d zero_utm) {
	pcl::PointCloud<PointT3>::Ptr pc_temp(new pcl::PointCloud<PointT3>);
	Eigen::Vector3d previous;
	int first = 1;
	for(std::vector<std::string>::const_iterator it = nd_refs.begin(); it != nd_refs.end(); ++it) {
		Node n = getNode(*it, nodes);
		Eigen::Vector3d pt_temp;
		pt_temp(0) = n.lat;
		pt_temp(1) = n.lon;
		pt_temp(2) = 0;
		if(first) {
			first = 0;
			previous = pt_temp;
		} else {
			*pc_temp += *(interpolate(toUtm(previous, zero_utm), toUtm(pt_temp, zero_utm)));
			previous = pt_temp;
		}

	}
	return pc_temp;
}

BuildingTools::Node BuildingTools::getNode(std::string nd_ref, std::vector< Node> nodes) {
	Node n_temp;
	n_temp.id = "";
	n_temp.lat = 0;
	n_temp.lon = 0;
	for(std::vector< Node>::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {
		if(nd_ref.compare(it->id) == 0) {
			n_temp = *it;
			break;
		}
	}
	return n_temp;
}

pcl::PointCloud<PointT3>::Ptr BuildingTools::interpolate(PointT3 a, PointT3 b) {
	// linear interpolation: return 1/sample_step points between a and b
	const float sample_step = 0.001;
	pcl::PointCloud<PointT3>::Ptr pc_temp(new pcl::PointCloud<PointT3>);

	for(float i=0;i<=1;i=i+sample_step) {
		PointT3 pt;

		pt.x = a.x + i*(b.x - a.x);
		pt.y = a.y + i*(b.y - a.y);
		pt.z = 0;

		pc_temp->push_back(pt);
	}
	return pc_temp;
}

std::string BuildingTools::downloadBuildings(double lat, double lon, double rad, std::string host) {
	std::string result;
	try {
		std::string url = host + "/api/interpreter?data=way[%27building%27](around:" + std::to_string(rad) + "," + std::to_string(lat) + "," + std::to_string(lon) + ");%20(._;%3E;);out;";
		std::cout << url << std::endl;

		curlpp::Easy request;
		curlpp::options::Url url_opt(url);

		// Setting the URL to retrive.
		request.setOpt(url_opt);

		std::ostringstream os;
		os << request;
		result = os.str();
	}
	catch ( curlpp::LogicError & e ) {
		std::cout << "curlpp logic error: " << e.what() << std::endl;
	}
	catch ( curlpp::RuntimeError & e ) {
		std::cout << "curlpp runtime error: " << e.what() << std::endl;
	}
	return result;
}

// toUtm converts to utm and already refer to zero_utm
PointT3 BuildingTools::toUtm(Eigen::Vector3d pt, Eigen::Vector3d zero_utm) {
	geographic_msgs::GeoPoint pt_lla;
	geodesy::UTMPoint pt_utm;

	pt_lla.latitude = pt(0);
	pt_lla.longitude = pt(1);
	pt_lla.altitude = 0;
	geodesy::fromMsg(pt_lla, pt_utm); 
	return PointT3((pt_utm.easting-zero_utm(0)), (pt_utm.northing-zero_utm(1)), 0);
}

std::vector<Building> BuildingTools::getBuildings(double lat, double lon, double rad, Eigen::Vector2d zero_utm, std::string host){
	std::string result = downloadBuildings(lat, lon, rad, host);
	Eigen::Vector3d v;
	v[0] = zero_utm[0];
	v[1] = zero_utm[1];
	v[2] = 0;
	std::vector<Building> tmp = parseBuildings(result,v);
	return tmp;
}

}
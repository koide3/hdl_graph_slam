#include <ros/ros.h>
#include <geodesy/utm.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>

#include <boost/format.hpp>
#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>

#include <hdl_graph_slam/building_tools.hpp>
#include <hdl_graph_slam/building_node.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace hdl_graph_slam {

class DeltaGraphSlamNodelet : public nodelet::Nodelet {
public:
    virtual void onInit() {
        nh = getNodeHandle();
        mt_nh = getMTNodeHandle();
        private_nh = getPrivateNodeHandle();

        navsat_sub = mt_nh.subscribe("/gps/navsat", 1024, &DeltaGraphSlamNodelet::navsat_callback, this);
        buildings_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/delta_graph_slam/buildings_cloud", 1);
    }

private:
    void navsat_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
        // std::cout << "LATITUDE: " << navsat_msg->latitude << std::endl;
        // std::cout << "LONGITUDE: " << navsat_msg->longitude << std::endl;
        // std::cout << "ALTITUDE: " << navsat_msg->altitude << std::endl << std::endl;

        // the first gps data position will be the origin of the map
        if(!zero_utm){
            geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
            gps_msg->header = navsat_msg->header;
            gps_msg->position.latitude = navsat_msg->latitude;
            gps_msg->position.longitude = navsat_msg->longitude;
            gps_msg->position.altitude = navsat_msg->altitude;
            
            // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
            geodesy::UTMPoint utm;
            geodesy::fromMsg(gps_msg->position, utm);
            Eigen::Vector2d xy(utm.easting, utm.northing);

            zero_utm = xy;
        }

        // download and parse buildings
        std::vector<Building> new_buildings = BuildingTools::getBuildings(navsat_msg->latitude, navsat_msg->longitude, 100, *zero_utm, "https://overpass-api.de");
        if(new_buildings.size() > 0) { 
            // inside here if there are buildings (new or already seen)
            std::cout << "We found buildings! " << std::endl;

            // buildingsCloud is the cloud containing all buildings
            pcl::PointCloud<PointT3>::Ptr buildingsCloud(new pcl::PointCloud<PointT3>);
            for(auto building = new_buildings.begin(); building != new_buildings.end(); building++){
                *buildingsCloud += *(building->geometry);
            }

            // publish buildings cloud
            sensor_msgs::PointCloud2Ptr buildingsCloud_msg(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*buildingsCloud, *buildingsCloud_msg);
            buildingsCloud_msg->header.frame_id = "map";
            buildingsCloud_msg->header.stamp = navsat_msg->header.stamp;
            buildings_pub.publish(buildingsCloud_msg);
        }
    }

    ros::NodeHandle nh;
    ros::NodeHandle mt_nh;
    ros::NodeHandle private_nh;

    ros::Subscriber navsat_sub;
    ros::Publisher buildings_pub;

    boost::optional<Eigen::Vector2d> zero_utm;

};
}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::DeltaGraphSlamNodelet, nodelet::Nodelet)
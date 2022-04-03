#include <ros/ros.h>
#include <geodesy/utm.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>

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

        navsat_sub = mt_nh.subscribe("/gps/navsat", 1, &DeltaGraphSlamNodelet::navsat_callback, this);
        buildings_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/delta_graph_slam/buildings_cloud", 1);
        markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/delta_graph_slam/markers", 16);
    }

private:
    void navsat_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
        // std::cout.precision(100);
        // std::cout << "LATITUDE: " << navsat_msg->latitude << std::endl;
        // std::cout << "LONGITUDE: " << navsat_msg->longitude << std::endl;
        // std::cout << "ALTITUDE: " << navsat_msg->altitude << std::endl << std::endl;

        // the first gps data position will be the origin of the map
        geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
        gps_msg->header = navsat_msg->header;
        gps_msg->position.latitude = navsat_msg->latitude;
        gps_msg->position.longitude = navsat_msg->longitude;
        gps_msg->position.altitude = navsat_msg->altitude;
        
        // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
        geodesy::UTMPoint utm;
        geodesy::fromMsg(gps_msg->position, utm);
        Eigen::Vector2d xy(utm.easting, utm.northing);

        if(!zero_utm){
            zero_utm = xy;

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

        // std::cout << "EASTING: " << xy.x() << std::endl;
        // std::cout << "NORTHING: " << xy.y() << std::endl << std::endl;

        if(!markers){
            markers.reset(new visualization_msgs::MarkerArray());
            markers->markers.resize(1);

            // gps markers
            visualization_msgs::Marker& traj_marker = markers->markers[0];
            traj_marker.header.frame_id = "map";
            traj_marker.header.stamp = ros::Time::now();
            traj_marker.ns = "gps";
            traj_marker.id = 0;
            traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

            traj_marker.pose.orientation.w = 1.0;
            traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;
        }

        visualization_msgs::Marker& traj_marker = markers->markers[0];
        int size = traj_marker.points.size()+1;
        traj_marker.points.resize(size);
        traj_marker.colors.resize(size);

        traj_marker.points[size-1].x = xy.x()-zero_utm->x();
        traj_marker.points[size-1].y = xy.y()-zero_utm->y();
        traj_marker.points[size-1].z = 0;

        traj_marker.colors[size-1].r = 1.0;
        traj_marker.colors[size-1].g = 1.0;
        traj_marker.colors[size-1].b = 1.0;
        traj_marker.colors[size-1].a = 1.0;

        markers_pub.publish(*markers);
        // std::cout << "X = " << xy.x()-zero_utm->x() << std::endl;
        // std::cout << "Y = " << xy.y()-zero_utm->y() << std::endl;
    }

    ros::NodeHandle nh;
    ros::NodeHandle mt_nh;
    ros::NodeHandle private_nh;

    ros::Subscriber navsat_sub;
    ros::Publisher buildings_pub;
    ros::Publisher markers_pub;

    boost::optional<Eigen::Vector2d> zero_utm;
    visualization_msgs::MarkerArrayPtr markers;


};
}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::DeltaGraphSlamNodelet, nodelet::Nodelet)
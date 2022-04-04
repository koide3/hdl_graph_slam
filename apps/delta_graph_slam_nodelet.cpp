#include <ros/ros.h>
#include <geodesy/utm.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
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
        gps_markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/delta_graph_slam/gps_markers", 16);
        bcloud_center_markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/delta_graph_slam/buildings_cloud_center_markers", 16);

        buildings_cloud_update_interval = private_nh.param<double>("buildings_cloud_update_interval", 20.0);
        buildings_publish_timer = mt_nh.createWallTimer(ros::WallDuration(buildings_cloud_update_interval), &DeltaGraphSlamNodelet::buildings_points_publish_timer_callback, this, false, false);
    
        gps_markers.reset(new visualization_msgs::MarkerArray());
        gps_markers->markers.resize(1);

        // gps markers
        visualization_msgs::Marker& traj_marker = gps_markers->markers[0];
        traj_marker.header.frame_id = "map";
        traj_marker.ns = "gps";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

        traj_marker.pose.orientation.w = 1.0;
        traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;
    }

    DeltaGraphSlamNodelet(): tfListener(tfBuffer) {}

private:
    void buildings_points_publish_timer_callback(const ros::WallTimerEvent& event){
        if(!zero_utm)
            return;

        geometry_msgs::TransformStamped transform_in_map;
        try {
            transform_in_map = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
        } catch(tf2::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
        }

        // convert from utm to lla
        geodesy::UTMPoint utm;

        // odom_msg->pose are the coords of current pose wrt zero_utm, so to get real coords we add zero_utm
        utm.easting = transform_in_map.transform.translation.x + zero_utm->x();
        utm.northing = transform_in_map.transform.translation.y + zero_utm->y();
        utm.altitude = 0;
        utm.zone = zero_utm_zone;
        utm.band = zero_utm_band;
        geographic_msgs::GeoPoint lla = geodesy::toMsg(utm);

        // visualize origin of map
        visualization_msgs::MarkerArray bcloud_center_markers;
        bcloud_center_markers.markers.resize(1);

        // marker of buildings pointcloud central point
        visualization_msgs::Marker& traj_marker = bcloud_center_markers.markers[0];
        traj_marker.header.stamp = ros::Time::now();
        traj_marker.header.frame_id = "map";
        traj_marker.ns = "buildings_cloud_center";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

        traj_marker.pose.orientation.w = 1.0;
        traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.7;

        traj_marker.points.resize(1);
        traj_marker.colors.resize(1);

        traj_marker.points[0].x = transform_in_map.transform.translation.x;
        traj_marker.points[0].y = transform_in_map.transform.translation.y;
        traj_marker.points[0].z = 0;

        traj_marker.colors[0].r = 0;
        traj_marker.colors[0].g = 0;
        traj_marker.colors[0].b = 1.0;
        traj_marker.colors[0].a = 1.0;

        bcloud_center_markers_pub.publish(bcloud_center_markers);

        // download and parse buildings
        std::vector<Building> new_buildings = BuildingTools::getBuildings(lla.latitude, lla.longitude, 40, *zero_utm, "https://overpass-api.de");
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
            buildingsCloud_msg->header.stamp = ros::Time::now();
            buildings_pub.publish(buildingsCloud_msg);
        }
    }

    void navsat_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {

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
            zero_utm_zone = utm.zone;
            zero_utm_band = utm.band;

            // first call without waiting timer
            ros::WallTimerEvent event;
            buildings_points_publish_timer_callback(event);
            buildings_publish_timer.start();
        }

        visualization_msgs::Marker& traj_marker = gps_markers->markers[0];
        traj_marker.header.stamp = ros::Time::now();

        int size = traj_marker.points.size()+1;
        traj_marker.points.resize(size);
        traj_marker.colors.resize(size);

        traj_marker.points[size-1].x = xy.x()-zero_utm->x();
        traj_marker.points[size-1].y = xy.y()-zero_utm->y();
        traj_marker.points[size-1].z = 0;

        traj_marker.colors[size-1].r = 1.0;
        traj_marker.colors[size-1].g = 1.0;
        traj_marker.colors[size-1].b = 1.0;
        traj_marker.colors[size-1].a = 0.2;

        gps_markers_pub.publish(*gps_markers);
    }

    ros::NodeHandle nh;
    ros::NodeHandle mt_nh;
    ros::NodeHandle private_nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    ros::Subscriber navsat_sub;
    ros::Publisher buildings_pub;
    ros::Publisher gps_markers_pub;
    ros::Publisher bcloud_center_markers_pub;


    double buildings_cloud_update_interval;
    ros::WallTimer buildings_publish_timer;

    boost::optional<Eigen::Vector2d> zero_utm;
    int zero_utm_zone;
    char zero_utm_band;
    visualization_msgs::MarkerArrayPtr gps_markers;
};
}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::DeltaGraphSlamNodelet, nodelet::Nodelet)
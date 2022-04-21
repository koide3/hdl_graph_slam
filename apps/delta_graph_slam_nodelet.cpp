#include <ros/ros.h>
#include <geodesy/utm.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <geographic_msgs/GeoPoint.h>
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

        tf_buffer.reset(new tf2_ros::Buffer());
        tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer));

        navsat_sub = mt_nh.subscribe("/gps/navsat", 1, &DeltaGraphSlamNodelet::navsat_callback, this);
        buildings_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/delta_graph_slam/buildings_cloud", 1);
        markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/delta_graph_slam/markers", 16);

        create_marker_array();

        double graph_update_interval = private_nh.param<double>("graph_update_interval", 3.0);
        optimization_timer = mt_nh.createWallTimer(ros::WallDuration(graph_update_interval), &DeltaGraphSlamNodelet::optimization_timer_callback, this);
    }

private:
    /**
     * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
     * @param event
     */
    void optimization_timer_callback(const ros::WallTimerEvent& event) {

        ros::WallTime start_, end_;
        start_ = ros::WallTime::now();

        create_buildings_pointcloud();

        end_ = ros::WallTime::now();
        // print results
        double execution_time = (end_ - start_).toNSec() * 1e-6;
        ROS_INFO_STREAM("Exectution time (ms): " << execution_time);

        if(markers_pub.getNumSubscribers()) {
            create_marker_array();
        }
    }

    /**
     * @brief download buildings from OSM and convert them into a point cloud
     * @return
     */
    void create_buildings_pointcloud() {

        // return if manager not initialized or cannot transform base_link
        if(!buildings_manager || !tf_buffer->canTransform("map", "base_link", ros::Time(0)))
            return;

        // use robot position to download buildings
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tf_buffer->lookupTransform("map", "base_link", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could NOT transform map to base_link: %s", ex.what());
            return;
        }

        // convert from utm to lla
        geodesy::UTMPoint utm;

        // odom_msg->pose are the coords of current pose wrt zero_utm, so to get real coords we add zero_utm
        utm.easting = transformStamped.transform.translation.x + zero_utm->x();
        utm.northing = transformStamped.transform.translation.y + zero_utm->y();
        utm.altitude = 0;
        utm.zone = zero_utm_zone;
        utm.band = zero_utm_band;
        geographic_msgs::GeoPoint lla = geodesy::toMsg(utm);

        // download and parse buildings
        std::vector<Building::Ptr> new_buildings = buildings_manager->getBuildings(lla.latitude, lla.longitude);

        // buildingsCloud is the cloud containing all buildings
        pcl::PointCloud<PointT3> buildingsCloud;
        for(Building::Ptr building : new_buildings){
            buildingsCloud += *(building->geometry);
        }

        // publish buildings cloud
        sensor_msgs::PointCloud2Ptr buildingsCloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(buildingsCloud, *buildingsCloud_msg);
        buildingsCloud_msg->header.frame_id = "map";
        buildingsCloud_msg->header.stamp = ros::Time::now();
        buildings_pub.publish(buildingsCloud_msg);
    }

    /**
     * @brief callback to store zero_utm and visualize gps positions
     * @param navsat_msg
     * @return
     */
    void navsat_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
        geographic_msgs::GeoPoint gps_msg;
        gps_msg.latitude = navsat_msg->latitude;
        gps_msg.longitude = navsat_msg->longitude;
        gps_msg.altitude = navsat_msg->altitude;
        
        // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
        geodesy::UTMPoint utm;
        geodesy::fromMsg(gps_msg, utm);
        Eigen::Vector2d xy(utm.easting, utm.northing);

        if(!zero_utm){
            zero_utm = xy;
            zero_utm_zone = utm.zone;
            zero_utm_band = utm.band;

            buildings_manager.reset(new BuildingTools("https://overpass-api.de", *zero_utm));
        }

        // add gps point to markers array
        visualization_msgs::Marker& gps_marker = markers->markers[0];
        gps_marker.header.stamp = ros::Time::now();

        int size = gps_marker.points.size()+1;
        gps_marker.points.resize(size);
        gps_marker.colors.resize(size);

        gps_marker.points[size-1].x = xy.x()-zero_utm->x();
        gps_marker.points[size-1].y = xy.y()-zero_utm->y();
        gps_marker.points[size-1].z = 0;

        gps_marker.colors[size-1].r = 1.0;
        gps_marker.colors[size-1].g = 1.0;
        gps_marker.colors[size-1].b = 1.0;
        gps_marker.colors[size-1].a = 0.2;
    }

    /**
     * @brief create visualization marker
     * @param stamp
     * @return
     */
    void create_marker_array() {
        if(!markers){
            markers.reset(new visualization_msgs::MarkerArray());
            markers->markers.resize(1);

            // gps markers
            visualization_msgs::Marker& gps_marker = markers->markers[0];
            gps_marker.header.frame_id = "map";
            gps_marker.ns = "gps";
            gps_marker.id = 0;
            gps_marker.type = visualization_msgs::Marker::SPHERE_LIST;

            gps_marker.pose.orientation.w = 1.0;
            gps_marker.scale.x = gps_marker.scale.y = gps_marker.scale.z = 0.5;
        }

        markers_pub.publish(*markers);        
    }

    ros::NodeHandle nh;
    ros::NodeHandle mt_nh;
    ros::NodeHandle private_nh;

    boost::shared_ptr<tf2_ros::Buffer> tf_buffer;
    boost::shared_ptr<tf2_ros::TransformListener> tf_listener;

    ros::Subscriber navsat_sub;
    ros::Publisher buildings_pub;
    ros::Publisher markers_pub;

    ros::WallTimer optimization_timer;

    boost::optional<Eigen::Vector2d> zero_utm;
    int zero_utm_zone;
    char zero_utm_band;

    visualization_msgs::MarkerArrayPtr markers;
    BuildingTools::Ptr buildings_manager;
};
}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::DeltaGraphSlamNodelet, nodelet::Nodelet)
// LLA -> local ENU conversion node, ported from LIO_SAM_6AXIS/src/simpleGpsOdom.cpp.
//
// Subscribes sensor_msgs/NavSatFix on `common/gps_topic` and republishes it as
// nav_msgs/Odometry on /gps_odom (ENU position). To keep a contract with the
// back-end, the original LLA is packed into covariance[1..3] and the NavSatFix
// variances into covariance[{0,7,14}]. covariance[{4,5,6}] carry status, num_sv
// and an orientation-valid flag.

#include <deque>
#include <mutex>

#include <Eigen/Core>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>

#include <GeographicLib/LocalCartesian.hpp>

class GNSSOdom
{
public:
    explicit GNSSOdom(ros::NodeHandle &nh_, const std::string &gps_topic,
                      const std::string &odom_frame)
        : nh(nh_), odom_link(odom_frame)
    {
        gpsSub = nh.subscribe(gps_topic, 1000, &GNSSOdom::GNSSCB, this,
                              ros::TransportHints().tcpNoDelay());
        odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 100, false);
        init_origin_pub = nh.advertise<nav_msgs::Odometry>("/init_odom", 10, true);
        path_pub = nh.advertise<nav_msgs::Path>("/gps_path", 100);

        yaw_quat = tf::createQuaternionMsgFromYaw(0.0);
        prev_pose.setZero();
    }

private:
    void GNSSCB(const sensor_msgs::NavSatFixConstPtr &msg)
    {
        if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) {
            return;
        }
        const Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);

        if (!initENU) {
            ROS_INFO("Init origin GPS LLA %f, %f, %f",
                     msg->latitude, msg->longitude, msg->altitude);
            geo_converter.Reset(lla[0], lla[1], lla[2]);
            initENU = true;

            nav_msgs::Odometry init_msg;
            init_msg.header.stamp = msg->header.stamp;
            init_msg.header.frame_id = odom_link;
            init_msg.child_frame_id = "gps";
            init_msg.pose.pose.position.x = lla[0];
            init_msg.pose.pose.position.y = lla[1];
            init_msg.pose.pose.position.z = lla[2];
            init_msg.pose.covariance[0] = msg->position_covariance[0];
            init_msg.pose.covariance[7] = msg->position_covariance[4];
            init_msg.pose.covariance[14] = msg->position_covariance[8];
            init_msg.pose.pose.orientation = yaw_quat;
            init_origin_pub.publish(init_msg);
            return;
        }

        double x, y, z;
        geo_converter.Forward(lla[0], lla[1], lla[2], x, y, z);
        const Eigen::Vector3d enu(x, y, z);
        if (std::abs(enu.x()) > 10000.0 || std::abs(enu.y()) > 10000.0 || std::abs(enu.z()) > 10000.0) {
            ROS_WARN("GPS ENU out of range: %f, %f, %f", enu(0), enu(1), enu(2));
            return;
        }

        bool orientationReady = false;
        const double d = std::sqrt(std::pow(enu(0) - prev_pose(0), 2) +
                                   std::pow(enu(1) - prev_pose(1), 2));
        if (d > 0.1) {
            const double yaw = std::atan2(enu(1) - prev_pose(1), enu(0) - prev_pose(0));
            yaw_quat = tf::createQuaternionMsgFromYaw(yaw);
            prev_pose = enu;
            orientationReady = true;
        }

        const int status = msg->status.status;
        const int satell_num = -1; // NavSatFix has no SV count field

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = odom_link;
        odom_msg.child_frame_id = "gps";
        odom_msg.pose.pose.position.x = enu(0);
        odom_msg.pose.pose.position.y = enu(1);
        odom_msg.pose.pose.position.z = enu(2);
        odom_msg.pose.covariance[0] = msg->position_covariance[0];
        odom_msg.pose.covariance[7] = msg->position_covariance[4];
        odom_msg.pose.covariance[14] = msg->position_covariance[8];
        odom_msg.pose.covariance[1] = lla[0];
        odom_msg.pose.covariance[2] = lla[1];
        odom_msg.pose.covariance[3] = lla[2];
        odom_msg.pose.covariance[4] = status;
        odom_msg.pose.covariance[5] = satell_num;
        odom_msg.pose.covariance[6] = orientationReady ? 1.0 : 0.0;
        odom_msg.pose.pose.orientation = yaw_quat;
        odom_pub.publish(odom_msg);

        path.header.frame_id = odom_link;
        path.header.stamp = msg->header.stamp;
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = enu(0);
        pose.pose.position.y = enu(1);
        pose.pose.position.z = enu(2);
        pose.pose.orientation = yaw_quat;
        path.poses.push_back(pose);
        path_pub.publish(path);
    }

    ros::NodeHandle nh;
    std::string odom_link;
    ros::Publisher odom_pub, path_pub, init_origin_pub;
    ros::Subscriber gpsSub;

    bool initENU = false;
    nav_msgs::Path path;
    GeographicLib::LocalCartesian geo_converter;
    Eigen::Vector3d prev_pose;
    geometry_msgs::Quaternion yaw_quat;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_gps_odom");
    ros::NodeHandle nh;

    std::string gps_topic, odom_frame;
    bool use_gps = false;
    nh.param<std::string>("common/gps_topic", gps_topic, "/gnss");
    nh.param<std::string>("common/odom_link", odom_frame, "camera_init");
    nh.param<bool>("common/useGPS", use_gps, false);
    if (!use_gps) {
        ROS_WARN("[simple_gps_odom] common/useGPS is false; node will idle.");
    }

    GNSSOdom gps(nh, gps_topic, odom_frame);
    ROS_INFO("\033[1;32m[simple_gps_odom] started. gps_topic=%s\033[0m", gps_topic.c_str());
    ros::spin();
    return 0;
}

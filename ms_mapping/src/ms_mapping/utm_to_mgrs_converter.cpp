#include "utm_to_mgrs_converter.h"
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geoid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <iostream>

GNSSStat convertUTM2MGRS(GNSSStat gnss_stat_utm, const MGRSPrecision precision) {
    constexpr int GZD_ID_size = 5;  // size of header like "53SPU"

    GNSSStat mgrs = gnss_stat_utm;
    mgrs.coordinate_system = CoordinateSystem::MGRS;
    try {
        std::string mgrs_code;
        GeographicLib::MGRS::Forward(
                gnss_stat_utm.zone, gnss_stat_utm.northup, gnss_stat_utm.x, gnss_stat_utm.y, gnss_stat_utm.latitude, static_cast<int>(precision), mgrs_code);
        mgrs.mgrs_zone = std::string(mgrs_code.substr(0, GZD_ID_size));
        mgrs.x = std::stod(mgrs_code.substr(GZD_ID_size, static_cast<int>(precision))) *
                 std::pow(10, static_cast<int>(MGRSPrecision::_1_METER) - static_cast<int>(precision));  // set unit as [m]
        mgrs.y = std::stod(mgrs_code.substr(GZD_ID_size + static_cast<int>(precision), static_cast<int>(precision))) *
                 std::pow(10, static_cast<int>(MGRSPrecision::_1_METER) - static_cast<int>(precision));  // set unit as [m]
        mgrs.z = gnss_stat_utm.z;                                 // set unit as [m]
    } catch (const GeographicLib::GeographicErr & err) {
        ROS_ERROR_STREAM("Failed to convert from UTM to MGRS: " << err.what());
    }
    return mgrs;
}

bool convertPCDToMGRS(const std::string& input_file_path, const std::string& output_file_path, 
                      double map_origin_northing, double map_origin_easting, double map_origin_height) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_file_path, *inputCloud) == -1) {
        ROS_ERROR_STREAM("Couldn't read file " << input_file_path);
        return false;
    }

    double counter = 0;
    GNSSStat gnss_stat_utm;
    GNSSStat gnss_stat_mgrs;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mgrs(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_mgrs->width = inputCloud->width;
    cloud_mgrs->height = inputCloud->height;
    cloud_mgrs->is_dense = inputCloud->is_dense;

    for (long unsigned int i = 0; i < inputCloud->points.size(); i++) {
        pcl::PointXYZI point = inputCloud->points[i];

        // convert local UTM to global UTM coordinates
        gnss_stat_utm.x = point.x + map_origin_easting;
        gnss_stat_utm.y = point.y + map_origin_northing;
        gnss_stat_utm.z = point.z + map_origin_height;
        gnss_stat_utm.coordinate_system = CoordinateSystem::UTM;
        gnss_stat_utm.zone = 35;
        gnss_stat_utm.northup = true;

        // convert latitude and longitude from UTM
        GeographicLib::UTMUPS::Reverse(gnss_stat_utm.zone, gnss_stat_utm.northup, gnss_stat_utm.x, gnss_stat_utm.y, gnss_stat_utm.latitude, gnss_stat_utm.longitude);

        gnss_stat_utm.altitude = gnss_stat_utm.z;

        // convert height from ellipsoid to orthometric
        double OrthometricHeight{0.0};
        try {
            GeographicLib::Geoid egm2008("egm2008-1");
            OrthometricHeight = egm2008.ConvertHeight(
                    gnss_stat_utm.latitude, gnss_stat_utm.longitude, gnss_stat_utm.altitude,
                    GeographicLib::Geoid::ELLIPSOIDTOGEOID);
        } catch (const GeographicLib::GeographicErr & err) {
            OrthometricHeight = gnss_stat_utm.altitude;
        }

        // convert global UTM to MGRS
        gnss_stat_mgrs = convertUTM2MGRS(gnss_stat_utm, MGRSPrecision::_100MICRO_METER);

        //create new pointcloud with mgrs coordinates
        pcl::PointXYZI point_mgrs;
        point_mgrs.x = gnss_stat_mgrs.x;
        point_mgrs.y = gnss_stat_mgrs.y;
        point_mgrs.z = OrthometricHeight;
        point_mgrs.intensity = point.intensity;
        cloud_mgrs->points.push_back(std::move(point_mgrs));

        if (counter == 1000000) {
            ROS_INFO("continue.");
            counter = 0;
        }
        counter++;
    }

    //save pcd file
    pcl::io::savePCDFileASCII(output_file_path, *cloud_mgrs);
    ROS_INFO_STREAM("Saved " << cloud_mgrs->points.size() << " data points to " << output_file_path);
    return true;
}

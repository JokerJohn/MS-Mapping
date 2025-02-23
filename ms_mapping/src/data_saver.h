
#ifndef FAST_LIO_SRC_PGO_SRC_DATASAVER_H_
#define FAST_LIO_SRC_PGO_SRC_DATASAVER_H_

#include "base_type.hpp"
#include "tic_toc.h"

using namespace std;
using namespace gtsam;

class DataSaver
{
public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        DataSaver();

        ~DataSaver();

        DataSaver(string _base_dir, string _sequence_name);

        void setDir(string _base_dir, string _sequence_name);

        void setConfigDir(string _configDirectory);

        void setMapDir(string _config_dir);

        void setKeyframe(bool _save_key_frame);

        void setExtrinc(bool _use_imu, bool _saveResultBodyFrame,
                        Eigen::Vector3d _t_body_sensor,
                        Eigen::Quaterniond _q_body_sensor);

        void saveOptimizedVerticesKITTI(gtsam::Values _estimates);

        void saveOptimizedVerticesKITTI(std::vector<Vector7> _estimates);

        void saveOdometryVerticesKITTI(std::string _filename);

        void saveOriginGPS(Eigen::Vector3d gps_point);

        void saveTimes(vector<double> keyframeTimes);

        void saveOptimizedVerticesTUM(gtsam::Values _estimates);

        void saveOptimizedVerticesTUM(std::vector<Vector7> _estimates);

        void saveOptimizedVerticesTUM(std::vector<Vector7> _estimates,
                                      std::string file_name);

        void saveOdomCov(std::vector<Eigen::Matrix<double, 6, 6>> &cov_vec,
                         std::string file_name);

        void saveOdometryVerticesTUM(
            std::vector<nav_msgs::Odometry> keyframePosesOdom);

        void saveEdgeErrors(const std::string &filename, gtsam::ISAM2 *isam, gtsam::Values &estimate);

        void saveGraphGtsam(gtsam::NonlinearFactorGraph gtSAMgraph,
                            gtsam::ISAM2 *isam, gtsam::Values isamCurrentEstimate);

        void saveGraph(std::vector<nav_msgs::Odometry> keyframePosesOdom);

        void saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
                           std::vector<nav_msgs::Odometry> updatedOdometryVec,
                           std::vector<sensor_msgs::PointCloud2> allResVec);

        void saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
                           std::vector<sensor_msgs::PointCloud2> allResVec);

        void saveLogBag(std::vector<Vector12> logVec);

        void writeDeskedFrame(pcl::PointCloud<PointT>::Ptr pc, int size);

        void saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
                           std::vector<pcl::PointCloud<PointT>::Ptr> allResVec);

        void saveLoopandImagePair(
            std::map<int, int> loopIndexCheckedMap,
            std::vector<std::vector<int>> all_camera_corre_match_pair);

        void savePointCloudMap(std::vector<nav_msgs::Odometry> allOdometryVec,
                               std::vector<pcl::PointCloud<PointT>::Ptr> allResVec);

        void savePointCloudMap(std::vector<nav_msgs::Odometry> allOdometryVec,
                               std::vector<pcl::PointCloud<PointT>::Ptr> allResVec, int index);

        void savePointCloudMap(std::vector<Eigen::Isometry3d> allOdometryVec,
                               std::vector<pcl::PointCloud<PointT>::Ptr> allResVec);

        void savePointCloudMapLIO(
            std::vector<nav_msgs::Odometry> allOdometryVec,
            std::vector<pcl::PointCloud<PointT>::Ptr> allResVec);

        void saveColorCloudMap(
            std::vector<nav_msgs::Odometry> allOdometryVec,
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> allResVec);

        void saveColorCloudMap(pcl::PointCloud<pcl::PointXYZRGB> cloud_ptr);

        void ReadPosesAndPointClouds(const std::string &tum_file, const std::string &cloud_directory,
                                     std::vector<Measurement> &measurements, pcl::PointCloud<PointT>::Ptr globalmap_ptr);

        gtsam::NonlinearFactorGraph BuildFactorGraph(const std::string &g2o_file, gtsam::Values &initial_values);

public:
        string save_directory, config_directory, map_directory;

private:
        string base_dir, sequence_name;
        string keyFrmaePath, keyColorFrmaePath;

        vector<string> configParameter;

        bool use_imu_frame = false;
        bool saveResultBodyFrame = false;
        bool save_key_frame = false;

        Eigen::Quaterniond q_body_sensor;
        Eigen::Vector3d t_body_sensor;

        vector<double> keyframeTimes;
};

#endif

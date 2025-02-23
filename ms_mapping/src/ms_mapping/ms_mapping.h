/*******************************************************
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 *
 * This file is part of FL2SAM (https://github.com/JokerJohn/FL2SAM-GPS).
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Xiangcheng Hu (xhubd@connect.ust.hk.com)
 * Date: ${Date}
 * Description:
 *******************************************************/
#ifndef SRC_POSE_SLAM_PRIOR_SRC_POSE_SLAM_PRIOR_H_
#define SRC_POSE_SLAM_PRIOR_SRC_POSE_SLAM_PRIOR_H_

// #include "base_type.hpp"
#include "../cloud_process.h"
#include "../data_saver.h"
// #include "tic_toc.h"

using namespace gtsam;
using namespace open3d;
typedef Eigen::Matrix<double, 2, 18> Vector28;

class MSMapping
{
public:
        MSMapping(ros::NodeHandle &nh)
        {
                LoadRosParams(nh);
                srvSaveMap = nh.advertiseService("/save_map", &MSMapping::SaveMap, this);
                pubLaserCloudSurround =
                    nh.advertise<sensor_msgs::PointCloud2>("/current_cloud", 10);
                pubLaserCloudCrop =
                    nh.advertise<sensor_msgs::PointCloud2>("/crop_cloud", 10);
                pubOdomAftPGO = nh.advertise<nav_msgs::Odometry>("/pgo_odom", 100);
                pubOdomAftGlobal = nh.advertise<nav_msgs::Odometry>("/global_odom", 100);
                pubPathAftPGO = nh.advertise<nav_msgs::Path>("/fusion_path", 100);
                pubPathNewpgo = nh.advertise<nav_msgs::Path>("/new_session_path", 100);
                pubPathLIO = nh.advertise<nav_msgs::Path>("/lio_path", 100);
                pubPathIMU = nh.advertise<nav_msgs::Path>("/imu_path", 100);
                pubLocalizationPath = nh.advertise<nav_msgs::Path>("/global_path", 100);
                pubMapAftPGO = nh.advertise<sensor_msgs::PointCloud2>("/pgo_map", 100);
                pubRGBVisualLoopConstraintEdge =
                    nh.advertise<visualization_msgs::MarkerArray>("/rgb_loop", 1);
                pubGlobalMapConstraintEdge =
                    nh.advertise<visualization_msgs::MarkerArray>("/map_constraint", 1);
                pubZUPTConstraintEdge =
                    nh.advertise<visualization_msgs::MarkerArray>("/zupt_constraint", 1);

                pubLoopConstraintEdge =
                    nh.advertise<visualization_msgs::MarkerArray>("/lio_loops", 1);
                pubSCLoopConstraintEdge =
                    nh.advertise<visualization_msgs::MarkerArray>("/sc_loop", 1);
                pubVisualLoopConstraintEdge =
                    nh.advertise<visualization_msgs::MarkerArray>("/intensity_loop", 1);
                pubLoopScanLocal =
                    nh.advertise<sensor_msgs::PointCloud2>("/loop_scan_local", 100);
                pubLoopSubmapLocal =
                    nh.advertise<sensor_msgs::PointCloud2>("/loop_submap_local", 100);
                pubLocalizationmap =
                    nh.advertise<sensor_msgs::PointCloud2>("/localization_map", 100);
                pubOldmap = nh.advertise<sensor_msgs::PointCloud2>("/old_map", 100);
                pubInitialCloud =
                    nh.advertise<sensor_msgs::PointCloud2>("/initial_cloud", 100);
                pubGpsConstraintEdge =
                    nh.advertise<visualization_msgs::MarkerArray>("/gps_constraints", 100);
                pubPoseBToMap = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                    "/base_link2map", 100);
                pubPoseOdomToMap = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                    "/odom2map", 100);
                pubDebugPoseOdom = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                    "/debug_odom_initial", 100);

                subImu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 2000,
                                                        &MSMapping::ImuCallback, this,
                                                        ros::TransportHints().tcpNoDelay());

                subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>(
                    "/cloud_registered_body", 10000, &MSMapping::LidarCallback, this);
                //                "/cloud_deskewed", 10000, &MSMapping::LidarCallback, this);

                //        subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>(
                //                "/cloud_effected", 10000, &MSMapping::LidarCallback, this);

                subLaserOdometry = nh.subscribe<nav_msgs::Odometry>(
                    "/lidar_odometry", 10000, &MSMapping::OdometryCallback, this);
                //                "/lio_sam_6axis/mapping/odometry", 10000, &MSMapping::OdometryCallback, this);

                subLaserOdometryIMU = nh.subscribe<nav_msgs::Odometry>(
                    "/lidar_odometry_current", 10000, &MSMapping::OdometryIMUCallback, this);

                subInitialPose = nh.subscribe("/initialpose", 10000,
                                              &MSMapping::InitialCallback, this);

                // load prior pose
                nh.param<vector<double>>("common/initial_pose", initial_pose_vector,
                                         vector<double>());

                InitParmeters();
        }

        ~MSMapping() {}

        void pose_slam();

        void LoopDetection();

        void VisualizationThread();

        void requestStop()
        {
                stopRequested = true;
        }

private:
        void InitParmeters();

        void InitSystem(Measurement &measurement);

        void ResetInitialization();

        void SaveCloud(pcl::PointCloud<PointT>::ConstPtr init_cloud, std::string name);

        void AddInitialPoseFactor(int closest_index, Measurement &measurement, const Pose3 &icp_pose);

        bool SyncGPS(std::deque<nav_msgs::Odometry> &gpsBuf,
                     nav_msgs::Odometry &aligedGps, double timestamp, double eps_cam);

        bool SyncData(Measurement &measurement);

        void AddOdomFactorToOldGraph();

        void AddMapPriorFactor();

        bool SaveKeyframeRadius(Pose6D &dtf);

        void AddNoMotionFactor();

        void AddLoopFactor();

        void AddGPSFactor();

        bool ZUPTDetector();

        void OptimizeGraph();

        void PerformRSLoopClosure(void);

        void SaveData();

        bool SaveMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        void GpsCallback(const nav_msgs::OdometryConstPtr &gps_msg_ptr);

        void LidarCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg_ptr);

        void OdometryCallback(const nav_msgs::OdometryConstPtr &odom_msg_ptr);

        void OdometryIMUCallback(const nav_msgs::OdometryConstPtr &odom_msg_ptr);

        void ImuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg);

        void InitialCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg_ptr);

        void LoopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr &loopMsg);

        void PubPath(void);

        void PubMap(void);

        void VisualizeLoopConstrains(std::map<int, int> loopMap, ros::Publisher &publisher, int type);

        void VisualizeLoopConstrains(std::map<int, std::vector<int>> loopMap, ros::Publisher &publisher, int type);

        bool FilterLoopPairs(int loopKeyCur, int loopKeyPre);

        bool DetectLoopClosureDistance(int *loopKeyCur, int *loopKeyPre);

        bool GetGlobalICP(Measurement &measurement_temp);

        void PublishPose(const ros::Time &time, const ros::Publisher &topic_pub,
                         const std::string &base_frame_id,
                         const Eigen::Matrix4d &transform_matrix);

        Vector28 GetStateFromLIO(const int node_id);

        State GetStateFromLIO2(const int node_id);

        inline float pointDistance(PointT p1, PointT p2)
        {
                return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
                            (p1.z - p2.z) * (p1.z - p2.z));
        }

        inline Eigen::Vector3d max(Eigen::Vector3d v1, Eigen::Vector3d v2)
        {
                return Eigen::Vector3d(std::max(v1.x(), v2.x()), std::max(v1.y(), v2.y()),
                                       std::max(v1.z(), v2.z()));
        }

        inline Eigen::Vector3d min(Eigen::Vector3d v1, Eigen::Vector3d v2)
        {
                return Eigen::Vector3d(std::min(v1.x(), v2.x()), std::min(v1.y(), v2.y()),
                                       std::min(v1.z(), v2.z()));
        }

        std::atomic<bool> stopRequested{false};

        ros::Subscriber subLaserCloudFullRes;
        ros::Subscriber subImu;
        ros::Subscriber subLaserOdometryIMU;
        ros::Subscriber subLaserOdometry;
        ros::Subscriber subGPS;
        ros::Subscriber subGPS2;
        ros::Subscriber subLoop;
        ros::Subscriber subRGBLoop;
        ros::Subscriber subLeftImages;
        ros::Subscriber subInitialPose;

        ros::Publisher pubMapAftPGO, pubOdomAftPGO, pubOdomAftGlobal;
        ros::Publisher pubPathLIO, pubPathAftPGO, pubLocalizationPath, pubPathBeforPGO, pubPathIMU, pubPathNewpgo;
        ros::Publisher pubLaserCloudSurround, pubLoopScanLocal, pubLoopSubmapLocal, pubGpsConstraintEdge;
        ros::Publisher pubLocalizationmap, pubInitialCloud, pubOldmap;
        ros::Publisher pubPoseOdomToMap, pubPoseBToMap, pubLaserCloudCrop, pubDebugPoseOdom;

        ros::Publisher pubLoopConstraintEdge, pubSCLoopConstraintEdge,
            pubGlobalMapConstraintEdge, pubVisualLoopConstraintEdge,
            pubRGBVisualLoopConstraintEdge, pubZUPTConstraintEdge;
        ros::Publisher pubLoopScanLocalRegisted, pubPathGnssCoords;
        ros::Publisher pubColorclouds, pubSemanticClouds;

        ros::ServiceServer srvSaveMap;

        std::mutex mKF;
        std::mutex mutexLock;
        std::mutex mutexSaveData;
        std::mutex mtxPosegraph;
        std::mutex mtxICP;
        std::mutex mtxLoopCheck;
        bool notified = false;

        Measurement measurement_curr;
        PointT lastGlobalPoint;

        PointT lastGPSPoint;
        map<int, int> gpsIndexContainer;
        map<int, int> mapIndexContainer;
        map<int, int> ZUPTIndexContainer;
        std::vector<Pose6D> keyframeGPS;
        Eigen::Isometry3d initialMatrix = Eigen::Isometry3d::Identity();

        bool useGpsElevation = true;
        bool isInitialized = false;
        bool poseReceived = false;
        Eigen::Vector3d init_gravity = Eigen::Vector3d::Zero();

        bool isNowKeyFrame = false;
        double movementAccumulation = 0.0;

        Eigen::Isometry3d gps_transform = Eigen::Isometry3d::Identity();

        double translationAccumulated = 1000000.0;
        double rotaionAccumulated = 1000000.0;

        bool aLoopIsClosed = false;
        bool aGlobalConstrained = false;
        bool initMaprTans = false;

        NavState propState;

        std::mutex mtxLoopInfo;
        std::mutex mtxRGBLoopInfo;

        Pose6D odom_pose_prev{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        Pose6D odom_pose_curr{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0.0;
        double t1_all = 0, t2_all = 0, t3_all = 0, t4_all = 0, t5_all = 0.0;

        // for loop closure detection
        int lastLoopIndex = -1;
        std::map<int, std::vector<int>> loopIndexCheckedMap;
        std::map<int, int> loopIndexSCcontainer;
        std::map<int, int> loopVisualIndexCheckedMap;
        std::map<int, int> loopRGBVisualIndexCheckedMap;

        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeHistoryKeyPoses;
        pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfFromMap;
        pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfFromLocalmap;
        Eigen::Matrix<float, 6, 6> icp_cov;

        int iterate_number = 0;
        Vector3d measured_gravity;
        std::mutex mtxLoopContainer;
        vector<pair<int, int>> loopIndexQueue;
        vector<gtsam::Pose3> loopPoseQueue;
        vector<double> loopTime;
        vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
        vector<noiseModel::Gaussian::shared_ptr> loopGaussianNoiseQueue;

        std::mutex imuLock;

        std::vector<gtsam::GPSFactor> keyframeGPSfactor;
        bool gpsTransfromInit = false;
        Eigen::Vector3d gpsLLA;
        int prev_node_idx = 0, curr_node_idx = 0;
        int prev_node_add_idx = 0, curr_node_add_idx = 0;
        const int node_rate = 1; // 10s jifenyici

        std::vector<Pose6D> keyframePoses2D;
        std::vector<Pose6D> keyframePoses3D;
        std::vector<Vector10> keyICPData;
        Vector10 curr_icp_data;
        std::vector<Vector10> keyTimeData;

        std::vector<Measurement> oldMeasurements;
        std::vector<Measurement> keyMeasures;
        std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
        std::queue<nav_msgs::Odometry::ConstPtr> odometryIMUBuf;
        std::deque<sensor_msgs::Imu> imuQueue;
        std::deque<nav_msgs::Odometry> gpsQueue;
        std::deque<nav_msgs::Odometry> gpsTempQueue;
        std::vector<nav_msgs::Odometry::ConstPtr> allOdometryVec;
        Matrix4d initialPose = Eigen::Matrix4d::Identity();
        double priorScore = 0.0;
        Vector28 lioState;
        State lioState2;
        State lioStateOld;
        std::vector<Vector28> keyLIOState;
        std::vector<State> keyLIOState2;
        std::vector<State> keyLIOStateOld;

        std::vector<PriorFactor<Pose3>> map_factors_vec;

        Pose6D curr_pose_initial;
        bool isDegenerate = false;
        Eigen::Vector3d eigen_values{0, 0, 0};
        Eigen::Vector3d eigen_values_ratio_rpy{0, 0, 0};
        Eigen::Vector3d eigen_values_ratio_xyz{0, 0, 0};

        int old_node_idx = 0;

        std::vector<double> initial_pose_vector;
        Eigen::Matrix4d priorPose = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d priorLIOPose = Eigen::Matrix4d::Identity();
        Eigen::Isometry3d mapTobase = Eigen::Isometry3d::Identity();

        double lastImuTime = -1;
        std::deque<sensor_msgs::Imu> last_imu_deque_;
        Measurement last_measurement;

        std::queue<std::pair<int, int>> candidate_loop_vec;
        std::queue<int> loop_type_vec;
        std::queue<double> loop_score_vec;

        std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
        std::unique_ptr<DataSaver> dataSaverPtr;

        pcl::PointCloud<PointT>::Ptr laserCloudMapPGO;
        pcl::PointCloud<PointT>::Ptr globalmap_ptr;
        pcl::PointCloud<PointT>::Ptr globalmap_filter_ptr;
        pcl::PointCloud<PointT>::Ptr globalmap_filter_extracted_ptr;

        std::vector<PointT> laserCloudOriSurfVec; // surf point holder for parallel computation
        std::vector<PointT> coeffSelSurfVec;
        std::vector<bool> laserCloudOriSurfFlag;

        double final_fitness = 0.0, prev_fitness = 0.0;
        double total_rmse = 0.0, prev_rmse = 0.0, curr_rmse = 0.0;
        double total_distance = 0.0, prev_distance = 0.0;
        double relative_rmse = 9999999999, relative_fitness = 1.0;

        pcl::VoxelGrid<PointT> downSizeFilterMapPGO;
        pcl::VoxelGrid<PointT> downSizeFilterICP;
        pcl::VoxelGrid<PointT> downSizeFilterScan;

        gtsam::NonlinearFactorGraph oldGraph;
        NonlinearFactorGraph newFactors;
        Values newValues;
        Values oldValues;
        ISAM2 *isam;
        Values currentEstimate;
        Eigen::MatrixXd xyzCovariance;
        Eigen::MatrixXd poseCovariance;

        Pose3 rel_pose;
        Pose3 predict_pose;

        Pose3 prevPose;
        Vector3 prevVel;
        NavState prevState;

        double zeroVelocitySigma = 0.001;
        double noMotionPositionSigma = 1e-3;
        double noMotionRotationSigma = 1e-4;
        double constantVelSigma = 0.01;

        Matrix6 pose_prior_covariance = Matrix6::Zero();

        //! No motion factors settings.
        gtsam::SharedNoiseModel zero_velocity_prior_noise_;
        gtsam::SharedNoiseModel no_motion_prior_noise_;
        gtsam::SharedNoiseModel constant_velocity_prior_noise_;

        noiseModel::Diagonal::shared_ptr priorPoseNoise;
        noiseModel::Diagonal::shared_ptr priorLIOPoseNoise;
        noiseModel::Diagonal::shared_ptr priorMapPoseNoise;
        noiseModel::Diagonal::shared_ptr robustLoopNoise;

        // between factor noise model
        SharedNoiseModel noise_odom_between;
        SharedGaussian priorMapPoseGaussianNoise;
        SharedGaussian LOOPGaussianNoise;

        Matrix4d imu2lidar_matrix = Matrix4d::Identity();
        Matrix4d lidar2imu_matrix = Matrix4d::Identity();

        CloudProcess cloud_process_;
};

#endif // SRC_POSE_SLAM_PRIOR_SRC_POSE_SLAM_PRIOR_H_

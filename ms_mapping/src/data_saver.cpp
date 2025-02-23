//
// Created by xchu on 2022/5/19.
//

#include "data_saver.h"

DataSaver::DataSaver() {}

DataSaver::~DataSaver() {}

DataSaver::DataSaver(string _base_dir, string _sequence_name)
{
    this->base_dir = _base_dir;
    this->sequence_name = _sequence_name;

    if (_base_dir.back() != '/')
    {
        _base_dir.append("/");
    }
    save_directory = _base_dir + sequence_name + '/';
    std::cout << "SAVE DIR:" << save_directory << std::endl;

    auto unused = system((std::string("exec rm -r ") + save_directory).c_str());
    unused = system((std::string("mkdir -p ") + save_directory).c_str());
}

void DataSaver::saveOptimizedVerticesTUM(gtsam::Values _estimates)
{
    std::fstream stream(save_directory + "optimized_odom_tum.txt",
                        std::fstream::out);
    stream.precision(15);
    for (int i = 0; i < _estimates.size(); i++)
    {
        auto &pose = _estimates.at(X(i)).cast<gtsam::Pose3>();
        //        auto &pose = _estimates.at<gtsam::Pose3>(X(i));
        gtsam::Point3 p = pose.translation();
        gtsam::Quaternion q = pose.rotation().toQuaternion();
        stream << keyframeTimes.at(i) << " " << p.x() << " " << p.y() << " "
               << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
               << q.w() << std::endl;
    }
}

void DataSaver::saveOptimizedVerticesTUM(std::vector<Vector7> _estimates)
{
    std::fstream stream(save_directory + "optimized_odom_tum.txt",
                        std::fstream::out);
    stream.precision(15);
    // x y z qx qy qz qw
    for (int i = 0; i < _estimates.size(); i++)
    {
        auto &pose = _estimates.at(i);
        stream << keyframeTimes.at(i) << " " << pose(0) << " " << pose(1) << " "
               << pose(2) << " " << pose(3) << " " << pose(4) << " " << pose(5)
               << " " << pose(6) << std::endl;
    }
}

void DataSaver::saveOptimizedVerticesTUM(std::vector<Vector7> _estimates,
                                         std::string file_name)
{
    std::fstream stream(save_directory + file_name, std::fstream::out);
    stream.precision(15);
    // x y z qx qy qz qw
    for (int i = 0; i < _estimates.size(); i++)
    {
        auto &pose = _estimates.at(i);
        stream << keyframeTimes.at(i) << " " << pose(0) << " " << pose(1) << " "
               << pose(2) << " " << pose(3) << " " << pose(4) << " " << pose(5)
               << " " << pose(6) << std::endl;
    }
}

void DataSaver::saveOdomCov(std::vector<Eigen::Matrix<double, 6, 6>> &cov_vec,
                            std::string file_name)
{

    std::fstream pcovfile(save_directory + file_name, std::fstream::out);
    pcovfile.precision(15);
    for (int i = 0; i < keyframeTimes.size(); i++)
    {
        pcovfile << keyframeTimes.at(i);
        Eigen::Matrix<double, 6, 6> poseCov = cov_vec.at(i);
        // write upper triangular part of the covariance matrix
        // in the order of translation followed by rotation
        for (int row = 0; row < 6; row++)
        {
            for (int col = row; col < 6; col++)
            {
                // swap the order of translation and rotation
                int swapped_row = (row < 3) ? (row + 3) : (row - 3);
                int swapped_col = (col < 3) ? (col + 3) : (col - 3);
                pcovfile << " " << poseCov(swapped_row, swapped_col);
            }
        }
        pcovfile << std::endl;
    }
    pcovfile.close();
}

void DataSaver::setDir(string _base_dir, string _sequence_name)
{
    this->base_dir = _base_dir;
    this->sequence_name = _sequence_name;

    if (_base_dir.back() != '/')
    {
        _base_dir.append("/");
    }
    save_directory = _base_dir + sequence_name + '/';

    auto unused = system((std::string("exec rm -r ") + save_directory).c_str());
    unused = system((std::string("mkdir -p ") + save_directory).c_str());

    //  LOG(INFO) << "SET DIR:" << save_directory;
}

void DataSaver::setConfigDir(string _config_dir)
{
    if (_config_dir.back() != '/')
    {
        _config_dir.append("/");
    }
    this->config_directory = _config_dir;
}

void DataSaver::setMapDir(string _config_dir)
{
    if (_config_dir.back() != '/')
    {
        _config_dir.append("/");
    }
    this->map_directory = _config_dir;
}

void DataSaver::setKeyframe(bool _save_key_frame)
{
    this->save_key_frame = _save_key_frame;

    if (save_key_frame)
    {
        keyFrmaePath = save_directory + "key_point_frame/";
        keyColorFrmaePath = save_directory + "key_color_frame/";
        string command = "mkdir -p " + keyFrmaePath;
        string command2 = "mkdir -p " + keyColorFrmaePath;
        system(command.c_str());
        system(command2.c_str());
        std::cout << "MKDIR NEW KEYFRAME DIR: " << command << std::endl;
        std::cout << "MKDIR NEW KEYFRAME DIR: " << command2 << std::endl;
    }
}

void DataSaver::setExtrinc(bool _use_imu, bool _saveResultBodyFrame,
                           Eigen::Vector3d _t_body_sensor,
                           Eigen::Quaterniond _q_body_sensor)
{
    this->use_imu_frame = _use_imu;
    this->saveResultBodyFrame = _saveResultBodyFrame;
    this->t_body_sensor = _t_body_sensor;
    this->q_body_sensor = _q_body_sensor;
}

void DataSaver::saveOptimizedVerticesKITTI(gtsam::Values _estimates)
{
    std::fstream stream(save_directory + "optimized_odom_kitti.txt",
                        std::fstream::out);
    stream.precision(15);
    //    for (const auto &key_value: _estimates) {
    //        auto p = dynamic_cast<const GenericValue<Pose3>
    //        *>(&key_value.value); if (!p) continue;
    //
    //        const Pose3 &pose = p->value();
    //
    //        Point3 t = pose.translation();
    //        Rot3 R = pose.rotation();
    //        auto col1 = R.column(1); // Point3
    //        auto col2 = R.column(2); // Point3
    //        auto col3 = R.column(3); // Point3
    //
    //        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " <<
    //        t.x() << " "
    //               << col1.y() << " " << col2.y() << " " << col3.y() << " " <<
    //               t.y() << " "
    //               << col1.z() << " " << col2.z() << " " << col3.z() << " " <<
    //               t.z() << std::endl;
    //    }

    for (int i = 0; i < _estimates.size(); i++)
    {
        auto &pose = _estimates.at(X(i)).cast<gtsam::Pose3>();
        //        gtsam::Point3 p = pose.translation();
        //        gtsam::Quaternion q = pose.rotation().toQuaternion();
        //        stream << keyframeTimes.at(i) << " " << p.x() << " " << p.y()
        //               << " " << p.z() << " "
        //               << q.x() << " " << q.y() << " "
        //               << q.z() << " " << q.w() << std::endl;

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1);
        auto col2 = R.column(2);
        auto col3 = R.column(3);

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
               << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
               << t.y() << " " << col1.z() << " " << col2.z() << " " << col3.z()
               << " " << t.z() << std::endl;
    }
}

void DataSaver::saveOptimizedVerticesKITTI(std::vector<Vector7> _estimates)
{
    std::fstream stream(save_directory + "optimized_odom_kitti.txt",
                        std::fstream::out);
    stream.precision(15);
    for (int i = 0; i < _estimates.size(); i++)
    {
        auto &pose = _estimates.at(i);

        Rot3 R(pose[6], pose[3], pose[4], pose[5]);
        auto col1 = R.column(1);
        auto col2 = R.column(2);
        auto col3 = R.column(3);

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << pose[0]
               << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
               << pose[1] << " " << col1.z() << " " << col2.z() << " " << col3.z()
               << " " << pose[2] << std::endl;
    }
}

void DataSaver::saveOdometryVerticesKITTI(std::string _filename)
{
    //  std::fstream stream(_filename.c_str(), std::fstream::out);
    //  stream.precision(15);
    //  for (const auto &_pose6d: keyframePoses) {
    //    gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
    //    Point3 t = pose.translation();
    //    Rot3 R = pose.rotation();
    //    auto col1 = R.column(1); // Point3
    //    auto col2 = R.column(2); // Point3
    //    auto col3 = R.column(3); // Point3
    //
    //    stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
    //    << " "
    //           << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y()
    //           << " "
    //           << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z()
    //           << std::endl;
    //  }
}

void DataSaver::saveOriginGPS(Eigen::Vector3d gps_point)
{
    std::fstream originStream(save_directory + "origin.txt", std::fstream::out);
    originStream.precision(15);
    originStream << gps_point[0] << " " << gps_point[1] << " " << gps_point[2]
                 << std::endl;
    originStream.close();
}

void DataSaver::saveTimes(vector<double> _keyframeTimes)
{
    if (_keyframeTimes.empty())
    {
        //    LOG(ERROR) << "EMPTY KEYFRAME TIMES!";
        return;
    }
    this->keyframeTimes = _keyframeTimes;

    std::fstream pgTimeSaveStream(save_directory + "times.txt",
                                  std::fstream::out);
    pgTimeSaveStream.precision(15);

    // save timestamp
    for (auto const timestamp : keyframeTimes)
    {
        pgTimeSaveStream << timestamp << std::endl; // path
    }

    pgTimeSaveStream.close();
}

void DataSaver::saveOdometryVerticesTUM(
    std::vector<nav_msgs::Odometry> keyframePosesOdom)
{
    std::fstream stream(save_directory + "odom_tum.txt", std::fstream::out);
    stream.precision(15);
    for (int i = 0; i < keyframePosesOdom.size(); i++)
    {
        nav_msgs::Odometry odometry = keyframePosesOdom.at(i);
        double time = odometry.header.stamp.toSec();
        // check the size of keyframeTimes
        stream << time << " " << odometry.pose.pose.position.x << " "
               << odometry.pose.pose.position.y << " "
               << odometry.pose.pose.position.z << " "
               << odometry.pose.pose.orientation.x << " "
               << odometry.pose.pose.orientation.y << " "
               << odometry.pose.pose.orientation.z << " "
               << odometry.pose.pose.orientation.w << std::endl;
    }
}


void DataSaver::saveEdgeErrors(const std::string &filename, gtsam::ISAM2 *isam, gtsam::Values &estimate)
{
    std::ofstream errorFile(save_directory + filename, std::fstream::out);
    if (!errorFile.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    std::cout << "save graph error file: " << filename << std::endl;

    // First, save all node poses
    for (const auto &key_value : estimate)
    {
        gtsam::Key key = key_value.key;
        //        if (gtsam::Symbol(key).chr() == 'X') {  // Assuming 'x' is used for pose variables
        size_t nodeIndex = gtsam::Symbol(key).index();
        gtsam::Pose3 pose = estimate.at<gtsam::Pose3>(key);
        gtsam::Point3 translation = pose.translation();
        gtsam::Quaternion rotation = pose.rotation().toQuaternion();
        errorFile << nodeIndex << " "
                  << translation.x() << " " << translation.y() << " " << translation.z() << " "
                  << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w()
                  << std::endl;
        //        }
    }

    // 保存每条边的误差
    for (const auto &factor : isam->getFactorsUnsafe())
    {
        if (auto f = dynamic_cast<gtsam::NoiseModelFactor *>(factor.get()))
        {
            Vector error = f->whitenedError(estimate);
            double weight = f->weight(estimate);
            //            errorFile << "Factor between: ";
            //            for (const auto& key : f->keys()) {
            //                errorFile << DefaultKeyFormatter(key) << " ";
            //            }
            //            errorFile << "Error: " << error.transpose()
            //                      << " Weight: " << weight << std::endl;
            for (const auto &key : f->keys())
            {
                // 提取节点序号
                size_t nodeIndex = Symbol(key).index();
                errorFile << nodeIndex << " ";
            }
            errorFile << error.transpose() << " ";

            // 检查是否为BetweenFactor<Pose3>类型
            if (auto betweenFactor = dynamic_cast<BetweenFactor<Pose3> *>(f))
            {
                Pose3 measured = betweenFactor->measured();
                Vector6 errorVector = error;
                //                errorFile << "Error (rotation then translation): "
                //                          << errorVector.head<3>().transpose() << " "  // 旋转误差
                //                          << errorVector.tail<3>().transpose() << " "; // 平移误差
                //                errorFile << "Measured: "
                //                          << measured.rotation().rpy().transpose() << " "  // 旋转测量
                //                          << measured.translation().transpose() << " ";    // 平移测量
                //                errorFile << measured.transpose() << " ";    // 平移测量
            }
            else
            {
                // prior factor
                // errorFile << "Error: " << error.transpose() << " ";
            }
            errorFile << weight << std::endl;
        }
    }
    errorFile.close();
}

void DataSaver::saveGraphGtsam(gtsam::NonlinearFactorGraph gtSAMgraph,
                               gtsam::ISAM2 *isam,
                               gtsam::Values isamCurrentEstimate)
{
    gtsam::writeG2o(gtSAMgraph, isamCurrentEstimate,
                    save_directory + "pose_graph.g2o");
    gtsam::writeG2o(isam->getFactorsUnsafe(), isamCurrentEstimate,
                    save_directory + "pose_graph.g2o");
    std::cout << "WRITE G2O FILE: " << save_directory + "pose_graph.g2o"
              << std::endl;
    std::cout << "Variable size: " << isamCurrentEstimate.size() << std::endl;
    std::cout << "Nonlinear factor size: " << isam->getFactorsUnsafe().size()
              << std::endl;

    // write pose cov
    if (this->keyframeTimes.empty())
    {
        std::cout << "Empty keyframeTimes: " << save_directory << ", Pls save keyframeTimes first!"
                  << std::endl;
        return;
    }

    std::fstream pcovfile(save_directory + "pose_cov.txt", std::fstream::out);
    pcovfile.precision(15);
    //     save timestamp
    for (int i = 0; i < keyframeTimes.size(); i++)
    {
        pcovfile << keyframeTimes.at(i);
        Matrix6 poseCov = isam->marginalCovariance(X(i));
        // write upper triangular part of the covariance matrix
        // in the order of translation followed by rotation
        for (int row = 0; row < 6; row++)
        {
            for (int col = row; col < 6; col++)
            {
                // swap the order of translation and rotation
                int swapped_row = (row < 3) ? (row + 3) : (row - 3);
                int swapped_col = (col < 3) ? (col + 3) : (col - 3);
                pcovfile << " " << poseCov(swapped_row, swapped_col);
            }
        }
        pcovfile << std::endl;
    }
    pcovfile.close();
}

void DataSaver::saveGraph(std::vector<nav_msgs::Odometry> keyframePosesOdom)
{
    std::fstream g2o_outfile(save_directory + "odom.g2o", std::fstream::out);
    g2o_outfile.precision(15);
    // g2o_outfile << std::fixed << std::setprecision(9);

    for (int i = 0; i < keyframePosesOdom.size(); i++)
    {
        nav_msgs::Odometry odometry = keyframePosesOdom.at(i);
        double time = odometry.header.stamp.toSec();

        g2o_outfile << "VERTEX_SE3:QUAT " << std::to_string(i) << " ";
        g2o_outfile << odometry.pose.pose.position.x << " ";
        g2o_outfile << odometry.pose.pose.position.y << " ";
        g2o_outfile << odometry.pose.pose.position.z << " ";
        g2o_outfile << odometry.pose.pose.orientation.x << " ";
        g2o_outfile << odometry.pose.pose.orientation.y << " ";
        g2o_outfile << odometry.pose.pose.orientation.z << " ";
        g2o_outfile << odometry.pose.pose.orientation.w << std::endl;
    }
    //  LOG(INFO) << "WRITE G2O VERTICES: " << keyframePosesOdom.size();
    g2o_outfile.close();
}

void DataSaver::saveLogBag(std::vector<Vector12> logVec)
{
    // save log files
    //    rosbag::Bag result_bag;
    //    result_bag.open(save_directory + sequence_name + "_log.bag",
    //    rosbag::bagmode::Write);
    //
    //    if (logVec.size()){
    //        ROS_ERROR("SAVE RESULT BAG FAILED, EMPTY!");
    //        return;
    //    }
    //
    //    for (int i = 0; i < logVec.size(); ++i) {
    //
    //    }
}

void DataSaver::saveResultBag(
    std::vector<nav_msgs::Odometry> allOdometryVec,
    std::vector<nav_msgs::Odometry> updatedOdometryVec,
    std::vector<sensor_msgs::PointCloud2> allResVec)
{
    rosbag::Bag result_bag;
    result_bag.open(save_directory + sequence_name + "_result.bag",
                    rosbag::bagmode::Write);

    std::cout << "save bias and velocity " << allOdometryVec.size() << ", "
              << updatedOdometryVec.size() << std::endl;
    for (int i = 0; i < allOdometryVec.size(); i++)
    {
        nav_msgs::Odometry _laserOdometry = allOdometryVec.at(i);
        result_bag.write("lio_odometry", _laserOdometry.header.stamp,
                         _laserOdometry);

        nav_msgs::Odometry updateOdometry = updatedOdometryVec.at(i);
        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
                         updateOdometry);

        //        sensor_msgs::PointCloud2 _laserCloudFullRes = allResVec.at(i);
        //        result_bag.write("cloud_deskewed",
        //        _laserCloudFullRes.header.stamp, _laserCloudFullRes);
    }
    std::cout << "save bias and velocity " << allOdometryVec.size() << std::endl;

    //    for (int i = 0; i < updatedOdometryVec.size(); i++) {
    //        nav_msgs::Odometry _laserOdometry = updatedOdometryVec.at(i);
    //        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
    //        _laserOdometry);
    //    }

    //    for (int i = 0; i < allResVec.size(); i++) {
    //        sensor_msgs::PointCloud2 _laserCloudFullRes = allResVec.at(i);
    //        result_bag.write("cloud_deskewed", _laserCloudFullRes.header.stamp,
    //        _laserCloudFullRes);
    //    }
    result_bag.close();
}

void DataSaver::saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
                              std::vector<sensor_msgs::PointCloud2> allResVec)
{
    rosbag::Bag result_bag;
    result_bag.open(save_directory + sequence_name + "_result.bag",
                    rosbag::bagmode::Write);

    std::cout << "odom and cloud size: " << allOdometryVec.size() << "--"
              << allResVec.size() << std::endl;

    if (allOdometryVec.size() == allOdometryVec.size())
    {
        ROS_ERROR("SAVE RESULT BAG FAILED");
        return;
    }

    //  LOG(INFO) << "ODOM AND PCD SIZE:" << allOdometryVec.size() << ", " <<
    //  allResVec.size();
    for (int i = 0; i < allOdometryVec.size(); i++)
    {
        nav_msgs::Odometry _laserOdometry = allOdometryVec.at(i);
        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
                         _laserOdometry);
    }

    for (int i = 0; i < allResVec.size(); i++)
    {
        sensor_msgs::PointCloud2 _laserCloudFullRes = allResVec.at(i);
        result_bag.write("cloud_deskewed", _laserCloudFullRes.header.stamp,
                         _laserCloudFullRes);
    }
    result_bag.close();
}

void DataSaver::saveResultBag(
    std::vector<nav_msgs::Odometry> allOdometryVec,
    std::vector<pcl::PointCloud<PointT>::Ptr> allResVec)
{
    rosbag::Bag result_bag;
    result_bag.open(save_directory + sequence_name + "_result.bag",
                    rosbag::bagmode::Write);

    //  LOG(INFO) << "ODOM AND PCD SIZE:" << allOdometryVec.size() << ", " <<
    //  allResVec.size();

    for (int i = 0; i < allResVec.size(); i++)
    {
        nav_msgs::Odometry _laserOdometry = allOdometryVec.at(i);
        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
                         _laserOdometry);

        sensor_msgs::PointCloud2 pointCloud2Msg;
        pcl::PointCloud<PointT>::Ptr _laserCloudFullRes = allResVec.at(i);
        pcl::toROSMsg(*_laserCloudFullRes, pointCloud2Msg);
        pointCloud2Msg.header = _laserOdometry.header;
        pointCloud2Msg.header.frame_id = "camera_init";
        result_bag.write("cloud_deskewed", _laserOdometry.header.stamp,
                         pointCloud2Msg);
    }
    result_bag.close();
    //  LOG(INFO) << "WRITE ROSBAG: " << save_directory + "_result.bag" << ",
    //  SIZE: " << result_bag.getSize();
}

void DataSaver::saveLoopandImagePair(
    std::map<int, int> loopIndexCheckedMap,
    std::vector<std::vector<int>> all_camera_corre_match_pair)
{
    std::ofstream loop_outfile;
    loop_outfile.open(save_directory + "lidar_loop.txt", std::ios::out);
    loop_outfile.precision(15);
    //  LOG(INFO) << "WRITE Lidar LOOP FILE: " << save_directory +
    //  "lidar_loop.txt";

    int j = 0;
    for (auto it = loopIndexCheckedMap.begin(); it != loopIndexCheckedMap.end();
         ++it)
    {
        int curr_node_idx = it->first;
        int prev_node_idx = it->second;

        //    geometry_msgs::Point p;
        //    p.x = keyframePosesUpdated[curr_node_idx].x;
        //    p.y = keyframePosesUpdated[curr_node_idx].y;
        //    p.z = keyframePosesUpdated[curr_node_idx].z;
        //
        //    p.x = keyframePosesUpdated[prev_node_idx].x;
        //    p.y = keyframePosesUpdated[prev_node_idx].y;
        //    p.z = keyframePosesUpdated[prev_node_idx].z;
        //
        //    // we can write some edges to g2o file
        //    //    g2o_out<<"EDGE_SE3:QUAT "<<curr_node_idx<<" "<<prev_node_idx<<"
        //    "
        //    //        <<p.x() <<" "<<p.y() <<" "<<p.z() <<" "
        //    //        <<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<" ";
        //
        //    if (saveLoopdata) {
        //      std::string common_name = std::to_string(curr_node_idx) + "_" +
        //      std::to_string(prev_node_idx);
        //
        //      std::string pcd_name_0 = common_name + "_0.pcd";
        //      std::string pcd_name_1 = common_name + "_1.pcd";
        //      pcl::io::savePCDFileBinary(pgScansDirectory + pcd_name_0,
        //      *keyframeLaserRawClouds[curr_node_idx]);
        //      pcl::io::savePCDFileBinary(pgScansDirectory + pcd_name_1,
        //      *keyframeLaserRawClouds[prev_node_idx]);
        //
        ////      cv::imwrite(pgImageDirectory + common_name + "_0_0.png",
        /// keyMeasures.at(curr_node_idx).camera0.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_1.png",
        /// keyMeasures.at(curr_node_idx).camera1.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_2.png",
        /// keyMeasures.at(curr_node_idx).camera2.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_3.png",
        /// keyMeasures.at(curr_node_idx).camera3.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_4.png",
        /// keyMeasures.at(curr_node_idx).camera4.front()); /      //
        /// cv::imshow("imgCallback", image_mat);
        ////
        ////      cv::imwrite(pgImageDirectory + common_name + "_1_0.png",
        /// keyMeasures.at(prev_node_idx).camera0.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_1.png",
        /// keyMeasures.at(prev_node_idx).camera1.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_2.png",
        /// keyMeasures.at(prev_node_idx).camera2.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_3.png",
        /// keyMeasures.at(prev_node_idx).camera3.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_4.png",
        /// keyMeasures.at(prev_node_idx).camera4.front());
        //      cv::imwrite(pgImageDirectory + common_name + "_1_4.png",
        //      resultMat_vec.at(j));
        //    }
        j++;
        loop_outfile.precision(15);
        loop_outfile << std::to_string(curr_node_idx) << " "
                     << std::to_string(prev_node_idx);
        loop_outfile << std::endl;
    }
    loop_outfile.close();
    //  LOG(INFO) << "SAVE LOOP FILE: " << loopIndexCheckedMap.size();

    // save camera pairs if their correspondences are sufficient
    std::ofstream camera_pair_outfile;
    camera_pair_outfile.open(save_directory + "camera_pair_indices.txt",
                             std::ios::out);
    //  LOG(INFO) << "WRITE CAMERA PAIR FILE: " << save_directory +
    //  "camera_pair_indices.txt"; LOG(INFO) << "Matching camera size: " <<
    //  all_camera_corre_match_pair.size();
    for (const auto &camera_pair : all_camera_corre_match_pair)
    {
        int lidar_idx_1 = camera_pair[0];
        int lidar_idx_2 = camera_pair[1];
        int cam_idx_1 = camera_pair[2];
        int cam_idx_2 = camera_pair[3];
        int num_corr = camera_pair[4];
        camera_pair_outfile << lidar_idx_1 << " " << lidar_idx_2 << " " << cam_idx_1
                            << " " << cam_idx_2 << " " << num_corr << std::endl;
    }
    camera_pair_outfile.close();
}

void DataSaver::savePointCloudMap(
    std::vector<nav_msgs::Odometry> allOdometryVec,
    std::vector<pcl::PointCloud<PointT>::Ptr> allResVec)
{
    std::cout << "odom and cloud size: " << allOdometryVec.size() << ", "
              << allResVec.size() << std::endl;
    if (allOdometryVec.size() != allResVec.size())
    {
        std::cout << "point cloud size do not equal to odom size!" << std::endl;
    }

    pcl::PointCloud<PointT>::Ptr laserCloudRaw(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr laserCloudTrans(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr traj_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    traj_cloud->width = allOdometryVec.size();
    traj_cloud->height = 1;
    traj_cloud->is_dense = false;

    for (int i = 0; i < allOdometryVec.size(); ++i)
    {
        nav_msgs::Odometry odom = allOdometryVec.at(i);
        laserCloudRaw = allResVec.at(i);

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.rotate(Eigen::Quaterniond(
            odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
        transform.pretranslate(Eigen::Vector3d(odom.pose.pose.position.x,
                                               odom.pose.pose.position.y,
                                               odom.pose.pose.position.z));
        pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans,
                                 transform.matrix());
        *globalmap += *laserCloudTrans;

        pcl::PointXYZ pt;
        pt.x = odom.pose.pose.position.x;
        pt.y = odom.pose.pose.position.y;
        pt.z = odom.pose.pose.position.z;
        traj_cloud->points.push_back(pt);

        if (save_key_frame)
        {
            std::cout << " process point cloud: " << i << "/" << allOdometryVec.size()
                      << std::endl;
            if (!laserCloudRaw->empty())
            {
                pcl::io::savePCDFileASCII(keyFrmaePath + std::to_string(i) + ".pcd",
                                          *laserCloudRaw);
            }
            else
                std::cout << "empty key frame!" << std::endl;
        }
    }

    std::cout << " save traj point cloud: " << traj_cloud->size() << std::endl;
    if (!traj_cloud->empty())
    {
        pcl::io::savePCDFileASCII(save_directory + "traj_pcd_lidar.pcd",
                                  *traj_cloud);
    }

    std::cout << " save global point cloud: " << globalmap->size() << std::endl;

    // save point cloud in lidar frame
    // if you want to save it in body frame(imu)
    // i will update it later
    if (!globalmap->empty())
    {
        globalmap->width = globalmap->points.size();
        globalmap->height = 1;
        globalmap->is_dense = false;

        // all cloud must rotate to body axis
        if (saveResultBodyFrame)
        {
            if (use_imu_frame)
            {
                try
                {
                    for (int j = 0; j < globalmap->points.size(); ++j)
                    {
                        PointT &pt = globalmap->points.at(j);
                        Eigen::Vector3d translation(pt.x, pt.y, pt.z);
                        translation = q_body_sensor * translation + t_body_sensor;

                        pt.x = translation[0];
                        pt.y = translation[1];
                        pt.z = translation[2];
                    }
                    pcl::io::savePCDFileASCII(save_directory + "global_pcd_imu.pcd",
                                              *globalmap);
                }
                catch (std::exception e)
                {
                    std::cerr << "save map falied! " << std::endl;
                }
            }
            else
            {
                pcl::io::savePCDFileASCII(save_directory + "global_pcd_lidar.pcd",
                                          *globalmap);
            }
        }
    }
    std::cout << "save global map finished! " << globalmap->size() << std::endl;
}

void DataSaver::savePointCloudMap(
    std::vector<nav_msgs::Odometry> allOdometryVec,
    std::vector<pcl::PointCloud<PointT>::Ptr> allResVec, int index)
{
    using namespace pcl;
    std::cout << "[save map]: odom-cloud-index size: " << allOdometryVec.size() << ", "
              << allResVec.size() << " " << index << std::endl;
    if (allOdometryVec.size() != allResVec.size())
    {
        std::cout << "point cloud size do not equal to odom size!" << std::endl;
    }

    PointCloud<PointT>::Ptr laserCloudRaw(new PointCloud<PointT>());
    PointCloud<PointT>::Ptr laserCloudTrans(new PointCloud<PointT>());
    PointCloud<PointT>::Ptr globalmap(new PointCloud<PointT>());
    PointCloud<PointXYZI>::Ptr traj_cloud(new PointCloud<PointXYZI>());
    //    traj_cloud->width = allOdometryVec.size();
    //    traj_cloud->height = 1;
    //    traj_cloud->is_dense = false;

    // begin from the index
    for (int i = index; i < allOdometryVec.size(); i++)
    {
        nav_msgs::Odometry odom = allOdometryVec.at(i);
        laserCloudRaw = allResVec.at(i);

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.rotate(Eigen::Quaterniond(
            odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
        transform.pretranslate(Eigen::Vector3d(odom.pose.pose.position.x,
                                               odom.pose.pose.position.y,
                                               odom.pose.pose.position.z));
        transformPointCloud(*laserCloudRaw, *laserCloudTrans, transform.matrix());
        *globalmap += *laserCloudTrans;
        laserCloudTrans->clear();

        //        PointXYZI pt;
        //        pt.x = odom.pose.pose.position.x;
        //        pt.y = odom.pose.pose.position.y;
        //        pt.z = odom.pose.pose.position.z;
        //        pt.intensity = 200;
        //        if (i >= index)
        //            pt.intensity = 100;
        //        traj_cloud->points.push_back(pt);

        if (i % 1000 == 0)
            std::cout << " process point cloud: " << i << "/" << allOdometryVec.size()
                      << std::endl;
        if (save_key_frame)
        {
            if (!laserCloudRaw->empty())
            {
                pcl::io::savePCDFileASCII(keyFrmaePath + std::to_string(i) + ".pcd", *laserCloudRaw);
            }
        }
    }

    //    if (!traj_cloud->empty()) {
    //        if (io::savePCDFileASCII(save_directory + "traj_pcd_lidar.pcd", *traj_cloud) == -1) {
    //            std::cout << "failed to save trajECORY point cloud: " << traj_cloud->size() << std::endl;
    //        } else {
    //            std::cout << " save traj point cloud success: " << traj_cloud->size() << std::endl;
    //        }
    //    }
    if (!globalmap->empty())
    {
        //        globalmap->width = globalmap->points.size();
        //        globalmap->height = 1;
        //        globalmap->is_dense = false;
        std::cout << "Save global point cloud: " << globalmap->size() << std::endl;

        // all cloud must rotate to body axis
        if (saveResultBodyFrame)
        {
            if (use_imu_frame)
            {
                for (int j = 0; j < globalmap->points.size(); ++j)
                {
                    PointT &pt = globalmap->points.at(j);
                    Eigen::Vector3d translation(pt.x, pt.y, pt.z);
                    translation = q_body_sensor * translation + t_body_sensor;
                    pt.x = translation[0];
                    pt.y = translation[1];
                    pt.z = translation[2];
                }
                io::savePCDFileASCII(save_directory + "final_map_imu.pcd", *globalmap);
            }
            else
            {
                io::savePCDFileASCII(save_directory + "final_map_lidar.pcd", *globalmap);
            }
        }
    }
    std::cout << "Save optimized map finished! " << globalmap->size() << std::endl;
}

void DataSaver::savePointCloudMap(
    std::vector<Eigen::Isometry3d> allOdometryVec,
    std::vector<pcl::PointCloud<PointT>::Ptr> allResVec)
{
    std::cout << "icp odom and cloud size: " << allOdometryVec.size() << ", "
              << allResVec.size() << std::endl;
    if (allOdometryVec.size() != allResVec.size())
    {
        std::cout << "icp point cloud size do not equal to odom size!" << std::endl;
    }

    pcl::PointCloud<PointT>::Ptr laserCloudRaw(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr laserCloudTrans(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr traj_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    traj_cloud->width = allOdometryVec.size();
    traj_cloud->height = 1;
    traj_cloud->is_dense = false;

    for (int i = 0; i < allOdometryVec.size(); ++i)
    {
        //    laserCloudRaw->clear();
        //    laserCloudTrans->clear();

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform = allOdometryVec.at(i);
        laserCloudRaw = allResVec.at(i);

        if (i % 100 == 0)
        {
            std::cout << "laser raw size: " << i << " " << laserCloudRaw->size()
                      << std::endl;
        }

        pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans,
                                 transform.matrix());

        if (i % 100 == 0)
        {
            std::cout << "laser trans size: " << i << " " << laserCloudTrans->size()
                      << " " << std::endl;
        }

        *globalmap += *laserCloudTrans;

        pcl::PointXYZ pt;
        pt.x = transform.translation().x();
        pt.y = transform.translation().y();
        pt.z = transform.translation().z();
        traj_cloud->points.push_back(pt);

        /*if (save_key_frame) {
          std::cout << " process point cloud: " << i << "/" << allOdometryVec.size()
                    << std::endl;
          if (!laserCloudRaw->empty()) {
            pcl::io::savePCDFileASCII(keyFrmaePath + std::to_string(i) + ".pcd",
                                      *laserCloudRaw);
          } else
            std::cout << "empty key frame " << i << std::endl;
        }*/
    }

    std::cout << "icp save traj point cloud: " << traj_cloud->size() << std::endl;
    if (!traj_cloud->empty())
    {
        pcl::io::savePCDFileASCII(save_directory + "traj_icp.pcd",
                                  *traj_cloud);
    }

    std::cout << "icp save global point cloud: " << globalmap->size()
              << std::endl;

    // save point cloud in lidar frame
    // if you want to save it in body frame(imu)
    // i will update it later
    if (!globalmap->empty())
    {
        globalmap->width = globalmap->points.size();
        globalmap->height = 1;
        globalmap->is_dense = false;

        // all cloud must rotate to body axis
        if (saveResultBodyFrame)
        {
            if (use_imu_frame)
            {
                try
                {
                    for (int j = 0; j < globalmap->points.size(); ++j)
                    {
                        PointT &pt = globalmap->points.at(j);
                        Eigen::Vector3d translation(pt.x, pt.y, pt.z);
                        translation = q_body_sensor * translation + t_body_sensor;

                        pt.x = translation[0];
                        pt.y = translation[1];
                        pt.z = translation[2];
                    }
                    pcl::io::savePCDFileASCII(save_directory + "map_icp.pcd",
                                              *globalmap);
                }
                catch (std::exception e)
                {
                    std::cerr << "save icp map falied! " << std::endl;
                }
            }
            else
            {
                pcl::io::savePCDFileASCII(save_directory + "map_icp.pcd",
                                          *globalmap);
            }
        }
    }
    std::cout << "icp save global map finished! " << globalmap->size()
              << std::endl;
}

void DataSaver::savePointCloudMapLIO(
    std::vector<nav_msgs::Odometry> allOdometryVec,
    std::vector<pcl::PointCloud<PointT>::Ptr> allResVec)
{
    std::cout << "lio odom and cloud size: " << allOdometryVec.size() << ", "
              << allResVec.size();
    if (allOdometryVec.size() != allResVec.size())
    {
        std::cout << "lio point cloud size do not equal to odom size!";
    }
    pcl::PointCloud<PointT>::Ptr laserCloudRaw(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr laserCloudTrans(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr traj_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    traj_cloud->width = allOdometryVec.size();
    traj_cloud->height = 1;
    traj_cloud->is_dense = false;

    for (int i = 0; i < allOdometryVec.size(); ++i)
    {
        nav_msgs::Odometry odom = allOdometryVec.at(i);
        laserCloudRaw = allResVec.at(i);

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.rotate(Eigen::Quaterniond(
            odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
        transform.pretranslate(Eigen::Vector3d(odom.pose.pose.position.x,
                                               odom.pose.pose.position.y,
                                               odom.pose.pose.position.z));

        pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans,
                                 transform.matrix());

        pcl::PointXYZ pt;
        pt.x = odom.pose.pose.position.x;
        pt.y = odom.pose.pose.position.y;
        pt.z = odom.pose.pose.position.z;
        traj_cloud->points.push_back(pt);
        /* if (save_key_frame) {
           std::cout << " process lio point cloud: " << i << "/"
                     << allOdometryVec.size() << std::endl;
           if (!laserCloudRaw->empty())
             pcl::io::savePCDFileASCII(
                 keyFrmaePath + std::to_string(i) + "_lidar_lio.pcd",
                 *laserCloudRaw);
         }*/
        *globalmap += *laserCloudTrans;
    }

    std::cout << " save lio traj point cloud: " << traj_cloud->size()
              << std::endl;
    if (!traj_cloud->empty())
    {
        pcl::io::savePCDFileASCII(save_directory + "traj_pcd_lidar_lio.pcd",
                                  *traj_cloud);
    }

    std::cout << " save lio global point cloud: " << globalmap->size()
              << std::endl;

    // save point cloud in lidar frame
    // if you want to save it in body frame(imu)
    // i will update it later
    if (!globalmap->empty())
    {
        globalmap->width = globalmap->points.size();
        globalmap->height = 1;
        globalmap->is_dense = false;

        // all cloud must rotate to body axis
        if (saveResultBodyFrame)
        {
            if (use_imu_frame)
            {
                try
                {
                    for (int j = 0; j < globalmap->points.size(); ++j)
                    {
                        PointT &pt = globalmap->points.at(j);
                        Eigen::Vector3d translation(pt.x, pt.y, pt.z);
                        translation = q_body_sensor * translation + t_body_sensor;

                        pt.x = translation[0];
                        pt.y = translation[1];
                        pt.z = translation[2];
                    }
                    pcl::io::savePCDFileASCII(save_directory + "map_lio.pcd",
                                              *globalmap);
                }
                catch (std::exception e)
                {
                    std::cerr << "save lio map falied! " << std::endl;
                }
            }
            else
            {
                pcl::io::savePCDFileASCII(save_directory + "map_lio.pcd",
                                          *globalmap);
            }
        }
    }
    std::cout << "save lio  map finished! " << globalmap->size()
              << std::endl;
}

void DataSaver::writeDeskedFrame(pcl::PointCloud<PointT>::Ptr pc, int index)
{
    std::string path = keyFrmaePath + std::to_string(index) + ".pcd";
    std::cout << " process point cloud: " << index << ", " << path << std::endl;
    if (!pc->empty())
        pcl::io::savePCDFileASCII(path.c_str(), *pc);
}

void DataSaver::saveColorCloudMap(
    std::vector<nav_msgs::Odometry> allOdometryVec,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> allResVec)
{
    std::cout << "odom and cloud size: " << allOdometryVec.size() << ", "
              << allResVec.size();

    if (allOdometryVec.size() != allResVec.size())
    {
        std::cout << "point cloud size do not equal to odom size!";
        //  return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudRaw(
        new pcl::PointCloud<pcl::PointXYZRGB>()); // giseop
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudTrans(
        new pcl::PointCloud<pcl::PointXYZRGB>()); // giseop
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalmap(
        new pcl::PointCloud<pcl::PointXYZRGB>()); // giseop
    for (int i = 0; i < allOdometryVec.size(); ++i)
    {
        laserCloudRaw->clear();
        laserCloudTrans->clear();
        nav_msgs::Odometry odom = allOdometryVec.at(i);
        laserCloudRaw = allResVec.at(i);

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.rotate(Eigen::Quaterniond(
            odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
        transform.pretranslate(Eigen::Vector3d(odom.pose.pose.position.x,
                                               odom.pose.pose.position.y,
                                               odom.pose.pose.position.z));

        pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans,
                                 transform.matrix());

        if (save_key_frame)
        {
            std::cout << " process rgb point cloud: " << i << "/"
                      << allOdometryVec.size() << std::endl;
            if (!laserCloudRaw->empty())
                pcl::io::savePCDFileASCII(
                    keyColorFrmaePath + std::to_string(i) + ".pcd", *laserCloudRaw);
        }
        *globalmap += *laserCloudTrans;
    }

    // save point cloud in lidar frame
    // if you want to save it in body frame(imu)
    // i will update it later
    if (!globalmap->empty())
    {
        globalmap->width = globalmap->points.size();
        globalmap->height = 1;
        globalmap->is_dense = false;

        // all cloud must rotate to body axis
        if (saveResultBodyFrame)
        {
            if (use_imu_frame)
            {
                for (int j = 0; j < globalmap->points.size(); ++j)
                {
                    pcl::PointXYZRGB &pt = globalmap->points.at(j);
                    Eigen::Vector3d translation(pt.x, pt.y, pt.z);
                    translation = q_body_sensor * translation + t_body_sensor;

                    pt.x = translation[0];
                    pt.y = translation[1];
                    pt.z = translation[2];
                }
                pcl::io::savePCDFileASCII(save_directory + "global_colormap_imu.pcd",
                                          *globalmap);
            }
            else
            {
                pcl::io::savePCDFileASCII(save_directory + "global_colormap_lidar.pcd",
                                          *globalmap);
            }
        }
        else
        {
            pcl::io::savePCDFileASCII(save_directory + "global_colormap_lidar.pcd",
                                      *globalmap);
        }
    }
    else
        std::cout << "EMPTY POINT CLOUD";
}

void DataSaver::saveColorCloudMap(pcl::PointCloud<pcl::PointXYZRGB> cloud_ptr)
{
    if (cloud_ptr.empty())
    {
        std::cout << "empty color map cloud!" << std::endl;
        return;
    }
    pcl::io::savePCDFileASCII(save_directory + "globalmap_color.pcd", cloud_ptr);
}

/*read parameter from kml_config.xml*/
int DataSaver::ReadParameter()
{
    xmlDocPtr pDoc = xmlReadFile((config_directory + "kml_config.xml").c_str(),
                                 "UTF-8", XML_PARSE_RECOVER);
    if (NULL == pDoc)
    {
        std::cout << "open config.xml error\n"
                  << std::endl;
        return 1;
    }

    xmlNodePtr pRoot = xmlDocGetRootElement(pDoc);
    if (NULL == pRoot)
    {
        std::cout << "get config.xml root error\n"
                  << std::endl;
        return 1;
    }

    xmlNodePtr pFirst = pRoot->children;

    while (NULL != pFirst)
    {
        xmlChar *value = NULL;
        if (!xmlStrcmp(pFirst->name, (const xmlChar *)("style")))
        {
            xmlNodePtr pStyle = pFirst->children;
            while (NULL != pStyle)
            {
                value = xmlNodeGetContent(pStyle);
                if (xmlStrcmp(pStyle->name, (const xmlChar *)("text")))
                {
                    configParameter.push_back((char *)value);
                }
                pStyle = pStyle->next;
            }
        }
        else if (!xmlStrcmp(pFirst->name, (const xmlChar *)("Placemark")))
        {
            xmlNodePtr pPlacemark = pFirst->children;
            while (NULL != pPlacemark)
            {
                value = xmlNodeGetContent(pPlacemark);
                if (xmlStrcmp(pPlacemark->name, (const xmlChar *)("text")))
                {
                    configParameter.push_back((char *)value);
                }
                pPlacemark = pPlacemark->next;
            }
        }
        else
        {
            value = xmlNodeGetContent(pFirst);
            if (xmlStrcmp(pFirst->name, (const xmlChar *)("text")))
            {
                configParameter.push_back((char *)value);
            }
        }
        pFirst = pFirst->next;
    }
    return 0;
}

int DataSaver::SaveKMLTrajectory(const std::vector<Eigen::Vector3d> lla_vec)
{
    if (1 == ReadParameter())
    {
        return 1;
    }

    // longitude, latitude, height
    std::fstream ofile(save_directory + "optimized_gps_trajectry.kml",
                       std::fstream::out);
    ofile.precision(15);

    int index = 0;
    ofile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl;
    ofile << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" << endl;
    ofile << "<Document>" << endl;
    ofile << "<name>"
          << "GPS Trajectory"
          << "</name>" << endl;
    ofile << "<description>"
          << "GPS Trajectory"
          << "</description>" << endl;

    ofile << "<Style id=\"" << configParameter[index++] << "\">" << endl;
    ofile << "<LineStyle>" << endl;
    ofile << "<color>"
          << "7fFF00FF"
          << "</color>" << endl;
    ofile << "<width>" << configParameter[index++] << "</width>" << endl;
    ofile << "</LineStyle>" << endl;
    ofile << "<PolyStyle>" << endl;
    ofile << "<color>"
          << "7fFF00FF"
          << "</color>" << endl;
    ofile << "</PolyStyle>" << endl;
    ofile << "</Style>" << endl;
    ofile << "<Placemark>" << endl;
    ofile << "<styleUrl>" << configParameter[index++] << "</styleUrl>" << endl;
    ofile << "<LineString>" << endl;
    ofile << "<extrude>" << configParameter[index++] << "</extrude>" << endl;
    ofile << "<tessellate>" << configParameter[index++] << "</tessellate>"
          << endl;
    ofile << "<altitudeMode>" << configParameter[index++] << "</altitudeMode>"
          << endl;
    ofile << "<coordinates>" << endl;

    for (int i = 0; i < lla_vec.size(); i++)
    {
        ofile << lla_vec.at(i)[1] << ',' << lla_vec.at(i)[0] << ','
              << lla_vec.at(i)[2] << endl;
    }

    ofile << "</coordinates>" << endl;
    ofile << "</LineString></Placemark>" << endl;
    ofile << "</Document></kml>" << endl;
    return 0;
}

int DataSaver::SaveKMLTrajectory(vector<pair<double, double>> WGSBL,
                                 vector<double> altitude,
                                 vector<pair<int, string>> segmentColor)
{
    int index = 0;

    if (1 == ReadParameter())
    {
        std::cout << "pls set kml config params first" << std::endl;
        return 1;
    }

    /*open kml file*/
    std::fstream ofile(save_directory + "optimized_path_gps.kml",
                       std::fstream::out);
    ofile.precision(15);

    ofile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl;
    ofile << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" << endl;
    ofile << "<Document>" << endl;
    ofile << "<name>"
          << "trajectory"
          << "</name>" << endl;
    ofile << "<description>"
          << "Optimized trajectory in GPS cordinate system"
          << "</description>" << endl;

    int indexCoor = 0;

    for (int i = 0; i < segmentColor.size(); i++)
    {
        index = 0;
        ofile << "<Style id=\"" << configParameter[index++] << "\">" << endl;
        ofile << "<LineStyle>" << endl;
        ofile << "<color>" << segmentColor[i].second << "</color>" << endl;
        ofile << "<width>" << configParameter[index++] << "</width>" << endl;
        ofile << "</LineStyle>" << endl;
        ofile << "<PolyStyle>" << endl;
        ofile << "<color>" << segmentColor[i].second << "</color>" << endl;
        ofile << "</PolyStyle>" << endl;
        ofile << "</Style>" << endl;
        ofile << "<Placemark>" << endl;
        ofile << "<styleUrl>" << configParameter[index++] << "</styleUrl>" << endl;
        ofile << "<LineString>" << endl;
        ofile << "<extrude>" << configParameter[index++] << "</extrude>" << endl;
        ofile << "<tessellate>" << configParameter[index++] << "</tessellate>"
              << endl;
        ofile << "<altitudeMode>" << configParameter[index++] << "</altitudeMode>"
              << endl;
        ofile << "<coordinates>" << endl;

        for (; indexCoor < segmentColor[i].first && index < altitude.size();
             indexCoor++)
        {
            ofile << WGSBL[indexCoor].second << ',' << WGSBL[indexCoor].first << ','
                  << altitude[indexCoor] << endl;
        }
        ofile << "</coordinates>" << endl;
        ofile << "</LineString></Placemark>" << endl;
    }
    ofile << "</Document></kml>" << endl;

    ofile.close();
    return 0;
}

void DataSaver::ReadPosesAndPointClouds(const std::string &tum_file, const std::string &cloud_directory,
                                        std::vector<Measurement> &measurements,  pcl::PointCloud<PointT>::Ptr globalmap_ptr)                                
{
    std::cout << "poses dir: " << tum_file << "\n"
              << cloud_directory << std::endl;

    globalmap_ptr->clear();
    pcl::VoxelGrid<PointT> downSizeFilterScan, downSizeFilterMapPGO;
    downSizeFilterScan.setLeafSize(scan_filter_size, scan_filter_size, scan_filter_size);
    downSizeFilterMapPGO.setLeafSize(0.5, 0.5, 0.5);
    // Read poses from TUM file
    std::ifstream tum_stream(tum_file);
    std::string line;
    while (std::getline(tum_stream, line))
    {
        if (line.empty())
        {
            std::cerr << "empty line: " << line << std::endl;
            break;
        }
        std::istringstream iss(line);
        double timestamp, x, y, z, qx, qy, qz, qw;
        if (!(iss >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw))
        {
            // 如果读取失败，可能是因为不完整的数据或格式错误，可以选择跳过或退出
            std::cerr << "Warning: Failed to parse line: " << line << std::endl;
            break; // 或者 break; 如果你想在这里结束循环
        }

        //        std::cout << "read pose: " << measurements.size() << " " << timestamp << " " << x << " " << y << " " << z
        //                  << std::endl;
        gtsam::Rot3 rot = gtsam::Rot3::Quaternion(qw, qx, qy, qz);
        gtsam::Point3 trans(x, y, z);
        gtsam::Pose3 gtpose(rot, trans);

        Pose6D pose = GtsamPose2Pose6D(gtpose);
        Measurement measurement;
        measurement.odom_time = timestamp;
        measurement.lidar_time = timestamp;
        measurement.key_pose = pose;
        measurement.updated_pose = pose;
        measurements.push_back(measurement);
    }
    std::cout << "measurements size: " << measurements.size() << std::endl;

    // 根据点云文件名读取点云数据
    TicToc ticToc;
    for (size_t index = 0; index < measurements.size(); index++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        std::string file_name = cloud_directory + std::to_string(index) + ".pcd";
        int read = pcl::io::loadPCDFile<pcl::PointXYZI>(file_name, *cloud);
        if (read == -1)
        {
            std::cerr << "Failed to read point cloud file: " << file_name << std::endl;
            continue;
        }
        if (!useRawCloud)
        {
            downSizeFilterScan.setInputCloud(cloud);
            downSizeFilterScan.filter(*cloud);
        }
        measurements[index].lidar = cloud;

        Pose3 pose = Pose6dTogtsamPose3(measurements.at(index).updated_pose);
        Eigen::Matrix4f transformation = pose.matrix().cast<float>();
        transformPointCloud(*cloud, *cloud, transformation);
        *globalmap_ptr += *cloud;
        if (index % 1000 == 0)
            std::cout << "read pcd: " << file_name << " " << index << std::endl;
    }
    downSizeFilterMapPGO.setInputCloud(globalmap_ptr);
    downSizeFilterMapPGO.filter(*globalmap_ptr);
    std::cout << BOLDBLUE << "globalmap size and time: " << globalmap_ptr->size() << std::endl;
    std::cout << BOLDBLUE << "READ PCD TIME: " << ticToc.toc() / 1000.0 << "s" << std::endl;
}

gtsam::NonlinearFactorGraph
DataSaver::BuildFactorGraph(const std::string &g2o_file, gtsam::Values &initial_values)
{
    using namespace gtsam;
    using Edge = std::pair<int, int>;
    using EdgeSet = std::unordered_set<Edge, boost::hash<Edge>>;
    std::cout << "Building graph from PGO file! " << std::endl;

    NonlinearFactorGraph graph;
    std::ifstream g2o_stream(g2o_file);
    std::string line;
    int line_number = 0;
    std::unordered_map<int, int> id_map, id_map_test; // 原始ID到新ID的映射
    int new_id = 0;                                   // 新ID
    EdgeSet existing_edges;

    // 噪声模型
    auto continuous_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(1e-4), Vector3::Constant(1e-3)).finished());
    //    auto non_continuous_noise = noiseModel::Diagonal::Sigmas(
    //            (Vector(6) << Vector3::Constant(1e-1), Vector3::Constant(1e-0)).finished());

    int num_continuous_edges = 0;
    int num_non_continuous_edges = 0;
    const int start_line = 0;
    //    const int start_line = 10; // 从第10行开始读取
    while (std::getline(g2o_stream, line))
    {
        std::istringstream iss(line);
        std::string tag;
        iss >> tag;
        if (tag == "VERTEX_SE3:QUAT")
        {
            int original_id;
            double x, y, z, qx, qy, qz, qw;
            iss >> original_id >> x >> y >> z >> qx >> qy >> qz >> qw;
            if (original_id < start_line && original_id >= 0)
                continue;

            // std::cout << "add node: " << original_id << " " << new_id << std::endl;
            Eigen::Quaterniond quat(qw, qx, qy, qz);
            Pose3 pose(Rot3(quat), Point3(x, y, z));
            // 为顶点创建一个新的ID并添加到映射中
            Symbol key = X(new_id);
            id_map[original_id] = new_id;
            initial_values.insert(key, pose);
            if (graph.size() == 0)
            {
                // 如果这是第一个处理的节点，添加一个先验因子
                graph.addPrior(key, pose, continuous_noise);
            }
            new_id++;
        }
        else if (tag == "EDGE_SE3:QUAT")
        {
            int id1, id2;
            double x, y, z, qx, qy, qz, qw;
            iss >> id1 >> id2 >> x >> y >> z >> qx >> qy >> qz >> qw;
            if ((id1 < start_line && id1 >= 0) || (id2 < start_line && id2 >= 0))
                continue;

            // 检查映射中是否存在这两个ID
            auto it1 = id_map.find(id1);
            auto it2 = id_map.find(id2);
            if (it1 == id_map.end() || it2 == id_map.end())
                continue; // 忽略未映射的边

            // set noise model
            Eigen::Matrix<double, 6, 6> information_matrix = Eigen::Matrix<double, 6, 6>::Zero();
            for (int i = 0; i < 6; ++i)
            {
                for (int j = i; j < 6; ++j)
                {
                    iss >> information_matrix(i, j);
                    if (i != j)
                    {
                        information_matrix(j, i) = information_matrix(i, j); // 填充对称部分
                    }
                }
            }

            Eigen::Quaterniond quat(qw, qx, qy, qz);
            Pose3 relative_pose(Rot3(quat), Point3(x, y, z));
            Symbol key1 = X(it1->second);
            Symbol key2 = X(it2->second);

            // 检查边是否已存在
            Edge edge(id1, id2);
            if (existing_edges.find(edge) != existing_edges.end())
            {
                std::cout << "repeat edge: " << it1->second << " -> " << it2->second << std::endl;
                std::cout << "repeat cov: " << information_matrix.inverse().diagonal().matrix().transpose()
                          << std::endl;
                continue; // 如果边已存在，跳过
            }
            existing_edges.insert(edge); // 添加新边到集合

            SharedNoiseModel noise_model = noiseModel::Gaussian::Covariance(information_matrix.inverse());
            // 根据是相邻节点还是非相邻节点选择不同的噪声模型
            bool is_continuous = (it1->second + 1) == it2->second;
            // auto noise_model = is_continuous ? continuous_noise : non_continuous_noise;
            graph.emplace_shared<BetweenFactor<Pose3>>(key1, key2, relative_pose, noise_model);

            if (is_continuous)
            {
                id_map_test[id1] = id2;
                num_continuous_edges++;
            }
            else
            {
                num_non_continuous_edges++;
            }
        }
    }
    std::cout << BOLDRED << "edge and loop numbers: " << num_continuous_edges << " " << num_non_continuous_edges << std::endl;
    std::cout << BOLDRED << "node numbers new_id: " << initial_values.size() - 1 << " " << new_id << std::endl;

    return graph;
}

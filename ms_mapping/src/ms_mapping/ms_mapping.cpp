#include "ms_mapping.h"
#include <opencv2/core/hal/intrin.hpp>

void MSMapping::pose_slam()
{
    // Continuously run SLAM until a stop is requested.
    while (!stopRequested.load())
    {
        TicToc tic_all;

        // Synchronize incoming data; if unavailable, wait briefly and retry.
        if (!SyncData(measurement_curr))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // Check system initialization.
        if (!isInitialized)
        {
            ROS_ERROR("Initialize system with %d lidar measurements", measurement_curr.lidar->size());
            std::cout << "[ERROR] System not initialized. Attempting initialization..." << std::endl;
            InitSystem(measurement_curr);
            continue;
        }

        // Update odometry: obtain the current pose and update the previous pose.
        Pose6D pose_curr = getOdom(measurement_curr.odom);
        odom_pose_prev = odom_pose_curr;
        odom_pose_curr = pose_curr;

        // Compute the relative transformation (delta) between consecutive poses.
        Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr);
        if (useKeyframe)
        {
            TicToc tic;

            // Measure the time for saving keyframe radius.
            if (!SaveKeyframeRadius(dtf))
                continue;
            t1 = tic.toc();
            t1_all += t1;
        }

        // Update the measurement with computed distance and key pose.
        measurement_curr.distance = sqrt(dtf.x * dtf.x + dtf.y * dtf.y + dtf.z * dtf.z);
        measurement_curr.key_pose = pose_curr;
        measurement_curr.updated_pose = pose_curr;

        // Lock keyframe data and update both keyMeasures and oldMeasurements.
        {
            std::unique_lock<std::mutex> kf_guard(mKF);
            keyMeasures.push_back(measurement_curr);     // New frame only.
            oldMeasurements.push_back(measurement_curr); // All frames.
            std::cout << "[INFO] Updated keyframe data. Total keyframes: " << keyMeasures.size() << std::endl;
        }

        // Update indices for the current and previous nodes.
        curr_node_idx = keyMeasures.size() - 1;
        prev_node_idx = curr_node_idx - 1;
        curr_node_add_idx = oldMeasurements.size() - 1;
        prev_node_add_idx = curr_node_add_idx - 1;

        // Debug info (preserved for future use):
        // std::cout << "Add measurements in old measurements: " << keyMeasures.size() << " " << oldMeasurements.size()
        //           << " " << prev_node_add_idx << "->" << curr_node_add_idx << " " << prev_node_idx << "->"
        //           << curr_node_idx << std::endl;
        // std::cout << "NODE DIFF: " << curr_node_add_idx - curr_node_idx << std::endl;

        // Retrieve the current state from LIO2 and update key state history.
        lioState2 = GetStateFromLIO2(curr_node_idx);
        keyLIOState2.push_back(lioState2);

        {
            // For baseline Frame-to-Frame (F2F)
            TicToc tic;
            AddOdomFactorToOldGraph();

            t2 = tic.toc();
            t2_all += t2;
        }

        // {
        //     // For baseline Map-to-Frame (M2F)
        //     TicToc tic;
        //     AddMapPriorFactor();
        //     std::cout << "[INFO] Global matching cost: " << tic.toc() << " ms" << std::endl;
        // }

        // Add loop closure factors if enabled.
        if (useLoopCloser)
        {
            AddLoopFactor();
        }

        {
            // Optimize the pose graph.
            TicToc tic;
            OptimizeGraph();
            t4 = tic.toc();
            t4_all += t4;
            std::cout << "[INFO] Pose graph optimized. t4: " << t4 << " ms" << std::endl;
        }

        // Publish the current path.
        PubPath();

        // Total time for this iteration.
        t5 = tic_all.toc();
        t5_all += t5;

        // Logging current and average times for each processing stage.
        if (curr_node_idx % 1 == 0)
        {
            std::cout << "[INFO] ----Frame: " << curr_node_idx
                      << ", Iteration Time: " << t5 << " ms, t1: " << t1
                      << " ms, t2: " << t2 << " ms, t3: " << t3
                      << " ms, t4: " << t4 << " ms" << std::endl;
            std::cout << "[INFO] ----Average Times: Iteration: " << t5_all / curr_node_idx
                      << " ms, t1: " << t1_all / curr_node_idx
                      << " ms, t2: " << t2_all / curr_node_idx << " ms, t3: " << t3_all / curr_node_idx
                      << " ms, t4: " << t4_all / curr_node_idx << " ms" << std::endl;
        }

        // Save ICP data and time information.
        curr_icp_data << eigen_values_ratio_xyz[0], eigen_values_ratio_xyz[1], eigen_values_ratio_xyz[2],
            eigen_values_ratio_rpy[0], eigen_values_ratio_rpy[1], eigen_values_ratio_rpy[2],
            iterate_number, total_rmse, final_fitness;
        keyICPData.push_back(curr_icp_data);
        measurement_curr.time_data << t1, t2, t3, t4, t5, t1_all, t2_all, t3_all, t4_all;

        // Throttle the loop briefly.
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void MSMapping::InitParmeters()
{
    // Load initial pose from the vector and print it.
    initialPose = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        initial_pose_vector.data(), 4, 4);
    std::cout << "[INFO] Loaded initial pose:\n"
              << initialPose.matrix() << std::endl;

    // Set up data saver parameters.
    dataSaverPtr = std::make_unique<DataSaver>(saveDirectory, sequence);
    dataSaverPtr->setExtrinc(useImuFrame, saveResultBodyFrame, t_body_sensor, q_body_sensor);
    dataSaverPtr->setConfigDir(configDirectory);
    dataSaverPtr->setMapDir(mapDirectory);
    dataSaverPtr->setKeyframe(saveKeyFrame);
    
    std::cout << "[INFO] Old map directory: " << mapDirectory << std::endl;

    // Set voxel filter leaf sizes.
    downSizeFilterMapPGO.setLeafSize(map_viewer_size, map_viewer_size, map_viewer_size);
    downSizeFilterScan.setLeafSize(scan_filter_size, scan_filter_size, scan_filter_size);
    downSizeFilterICP.setLeafSize(scan_filter_size * 2, scan_filter_size * 2, scan_filter_size * 2);

    // Initialize various point cloud and KD-tree pointers.
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointT>());
    kdtreeSurfFromLocalmap.reset(new pcl::KdTreeFLANN<PointT>());
    laserCloudMapPGO.reset(new pcl::PointCloud<PointT>());
    globalmap_ptr.reset(new pcl::PointCloud<PointT>());
    globalmap_filter_ptr.reset(new pcl::PointCloud<PointT>());
    globalmap_filter_extracted_ptr.reset(new pcl::PointCloud<PointT>());

    // Optionally load a global prior map.
    /*
    if (useGlobalPrior) {
        TicToc ticToc;
        pcl::io::loadPCDFile((configDirectory + "map_file/" + sequence + ".pcd").c_str(), *globalmap_ptr);
        std::cout << BOLDWHITE << "Load map file: " << configDirectory + sequence + ".pcd"
                  << ", " << globalmap_ptr->size() << std::endl;
        if (globalmap_ptr->empty()) {
            std::cout << BOLDRED << "Failed to load empty map!" << std::endl;
            ros::shutdown();
            return;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*globalmap_ptr, *global_map);
        // VoxelGrid downsampling
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(global_map);
        sor.setLeafSize(1.0, 1.0, 1.0);
        // sor.setLeafSize(0.1, 0.1, 0.1);
        sor.filter(*globalmap_filter_ptr);
        kdtreeSurfFromMap->setInputCloud(globalmap_filter_ptr);
        std::cout << BOLDBLUE << "Global map size and time: " << globalmap_filter_ptr->size() << ", "
                  << ticToc.toc() << "ms" << std::endl;
    }
    */

    // For multi-session mapping mode, load the global map for relocalization.
    if (useMultiMode)
    {
        TicToc ticToc;
        pcl::io::loadPCDFile((mapDirectory + "map.pcd").c_str(), *globalmap_ptr);
        std::cout << BOLDWHITE << "Load map file: " << mapDirectory + "map.pcd"
                  << ", " << globalmap_ptr->size() << std::endl;
        if (globalmap_ptr->empty())
        {
            std::cout << BOLDRED << "Failed to load empty map!" << std::endl;
            ros::shutdown();
            return;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*globalmap_ptr, *global_map);
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(global_map);
        sor.setLeafSize(1.0, 1.0, 1.0);
        sor.filter(*globalmap_filter_ptr);
        kdtreeSurfFromMap->setInputCloud(globalmap_filter_ptr);
        std::cout << BOLDBLUE << "Global map size and time: " << globalmap_filter_ptr->size() << ", "
                  << ticToc.toc() << "ms" << std::endl;

        // Check for consistency between old graph and measurements.
        TicToc ticToc2;
        oldGraph = dataSaverPtr->BuildFactorGraph(mapDirectory + "pose_graph.g2o", oldValues);
        old_node_idx = oldValues.size();
        dataSaverPtr->ReadPosesAndPointClouds(mapDirectory + "optimized_poses_tum.txt", mapDirectory + "key_point_frame/", oldMeasurements, globalmap_ptr);
        if (oldValues.size() != oldMeasurements.size())
        {
            std::cout << BOLDRED << "Inconsistency found: graph size (" << oldGraph.size()
                      << ") does not match measurements size (" << oldMeasurements.size() << ")" << std::endl;
        }
        std::cout << "[INFO] Prior measurements: poses " << oldMeasurements.size() << ", graph "
                  << oldGraph.size() << ", values " << oldValues.size() << std::endl;
        std::cout << "[INFO] Read data time: " << ticToc2.toc() / 1000.0 << " s" << std::endl;
    }

    // Initialize ISAM2 parameters.
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    // Set noise models for prior pose.
    Vector priorPoseNoiseVector6(6);
    priorPoseNoiseVector6 << 1e0, 1e0, 1e0, 1e2, 1e2, 1e2;
    priorPoseNoise = gtsam::noiseModel::Diagonal::Variances(priorPoseNoiseVector6);

    // Set noise models for LIO pose.
    Vector priorLIOPoseNoiseVector6(6);
    priorLIOPoseNoiseVector6 << 1e0, 1e0, 1e0, 1e2, 1e2, 1e2;
    priorLIOPoseNoise = noiseModel::Diagonal::Variances(priorLIOPoseNoiseVector6);

    // Set odometry noise model.
    Vector odomNoiseVector6(6);
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    noise_odom_between = noiseModel::Diagonal::Variances(odomNoiseVector6);

    // Set global map noise model.
    Vector priorMapPoseNoiseVector6(6);
    priorMapPoseNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2;
    priorMapPoseNoise = noiseModel::Diagonal::Variances(priorMapPoseNoiseVector6);

    // Set zero velocity and no-motion factor noise models.
    zero_velocity_prior_noise_ = noiseModel::Isotropic::Sigma(3u, zeroVelocitySigma);
    Vector6 sigmas;
    sigmas.head<3>().setConstant(noMotionPositionSigma);
    sigmas.tail<3>().setConstant(noMotionRotationSigma);
    no_motion_prior_noise_ = noiseModel::Diagonal::Sigmas(sigmas);
}

void MSMapping::InitSystem(Measurement &measurement)
{
    using namespace open3d;
    std::cout << "[INFO] MS-Mapping: Waiting for initial pose..." << std::endl;

    // If multi-session mapping is not enabled, use identity pose.
    if (!useMultiMode)
    {
        initialPose = Eigen::Matrix4d::Identity();
        isInitialized = true;
        std::cout << "[WARN] Multi-session mode disabled. Using identity pose." << std::endl;
        return;
    }

    // Publish the old map point cloud.
    std::cout << "[INFO] Publishing old session map point cloud..." << std::endl;
    publishCloud(pubOldmap, globalmap_ptr, ros::Time::now(), odom_link);

    // Convert and save the initial cloud.
    pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
    std::shared_ptr<geometry::PointCloud> source_o3d = cloud_process_.GetO3dPointCloudFromPCL(*measurement.lidar);
    SaveCloud(measurement.lidar, "_init.pcd");

    TicToc ticToc;
    // Find the closest keyframe and create the local map.
    int closest_index = -1;
    auto local_map = cloud_process_.CreateLocalMap(initialPose, oldValues, mapDirectory + "key_point_frame/", closest_index, 100);
    std::cout << "[INFO] Publishing local map for localization..." << std::endl;
    publishCloud(pubLocalizationmap, local_map, ros::Time().fromSec(measurement.odom_time), odom_link);

    // Convert and save the local map point cloud.
    std::shared_ptr<open3d::geometry::PointCloud> target_o3d = cloud_process_.GetO3dPointCloudFromPCL(*local_map);
    SaveCloud(local_map, "_localmap.pcd");

    // Initialize ICP transformation.
    Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
    pipelines::registration::RegistrationResult icp;
    auto criteria = pipelines::registration::ICPConvergenceCriteria(20);

    if (initialPose != Eigen::Matrix4d::Identity())
    {
        std::cout << "[INFO] Received initial pose from YAML file:" << std::endl
                  << initialPose.matrix() << std::endl;
        // Perform ICP registration using the selected method.
        icp = cloud_process_.PerformICPRegistration(source_o3d, target_o3d, initialPose, criteria, method);
        trans = icp.transformation_;
        double score = icp.inlier_rmse_;
        double overlap = icp.fitness_;
        Pose3 icp_pose = Matrix4dToPose3(trans);
        *unused_result = *TransformPointCloud(measurement.lidar, trans);
        publishCloud(pubInitialCloud, unused_result, ros::Time().fromSec(measurement.odom_time), odom_link);
        // Check the ICP registration result.
        if (!cloud_process_.CheckICPResult(score, overlap))
        {
            std::cout << "[ERROR] ICP registration failed. Resetting initialization." << std::endl;
            ResetInitialization();
            return;
        }
        ROS_WARN("SYSTEM INIT SUCCESS!!!");

        // Compute the ICP covariance matrix.
        initialPose = priorPose = trans.matrix().cast<double>();
        priorScore = score;
        std::cout << "[INFO] Initial pose:" << std::endl
                  << initialPose.matrix() << std::endl;
        isInitialized = true;

        // Add initial pose factor to the old graph.
        AddInitialPoseFactor(closest_index, measurement, icp_pose);
        return;
    }
    else
    {
        // If an initial pose is received from RVIZ.
        if (poseReceived)
        {
            icp = cloud_process_.PerformICPRegistration(source_o3d, target_o3d, initialPose, criteria, method);
            trans = icp.transformation_;
            double score = icp.inlier_rmse_;
            double overlap = icp.fitness_;
            // Update global pose and score in the measurement.
            measurement.global_pose = Matrix2Pose6D(trans);
            measurement.global_score = score;
            *unused_result = *TransformPointCloud(measurement_curr.lidar, trans);
            publishCloud(pubInitialCloud, unused_result, ros::Time::now(), odom_link);
            if (!cloud_process_.CheckICPResult(score, overlap))
            {
                std::cout << "[ERROR] ICP registration from RVIZ pose failed." << std::endl;
                poseReceived = false;
                isInitialized = false;
                return;
            }
            priorPose = trans.matrix();
            PublishPose(ros::Time::now(), pubPoseOdomToMap, odom_link, priorPose.matrix());
            isInitialized = true;
            std::cout << "[INFO] Initial pose from RVIZ:" << std::endl
                      << priorPose.matrix() << std::endl;
            std::cout << "[INFO] Initial LIO pose:" << std::endl
                      << Pose6D2Matrix(getOdom(measurement_curr.odom)).matrix() << std::endl;
            return;
        }
    }
}

void MSMapping::ResetInitialization()
{
    // Reset the system initialization to default values.
    priorPose = initialPose = Eigen::Matrix4d::Identity();
    priorScore = loopFitnessScoreThreshold;
    isInitialized = false;
    std::cout << "[INFO] System initialization reset to default (identity pose)." << std::endl;
}

void MSMapping::SaveCloud(pcl::PointCloud<PointT>::ConstPtr init_cloud, std::string name)
{
    std::string dir_path = saveDirectory + sequence + "/";
    std::string full_path = dir_path + sequence + name;
    pcl::io::savePCDFileASCII(full_path, *init_cloud);
    std::cout << "[INFO] Saved initial cloud to: " << full_path << std::endl;
}

//---------------------------------------------------------------------
// Add Initial Pose Factor to Graph
//---------------------------------------------------------------------
void MSMapping::AddInitialPoseFactor(int closest_index, Measurement &measurement, const Pose3 &icp_pose)
{
    int c_index = oldValues.size();
    int p_index = closest_index;
    Pose3 prev_pose3 = Pose6dTogtsamPose3(oldMeasurements.at(p_index).updated_pose);
    Pose6D local_pose3d = getOdom(measurement.odom);
    Pose3 local_pose = Pose6dTogtsamPose3(local_pose3d);
    Pose3 curr_pose3 = icp_pose.compose(local_pose);

    if (!oldValues.exists(X(c_index)))
    {
        oldValues.insert(X(c_index), curr_pose3);
    }
    PublishPose(ros::Time().fromSec(measurement.odom_time), pubPoseOdomToMap, odom_link, curr_pose3.matrix());
    std::cout << BOLDRED << "[INFO] Local pose: " << local_pose.translation().transpose() << std::endl;
    std::cout << "[INFO] Current ICP pose: " << curr_pose3.translation().transpose() << std::endl;
    // std::cout << "Current ICP cov: " << icp_cov.diagonal().transpose() * 1e6 << std::endl;
    // std::cout << "Local pose cov: " << local_pose_cov.diagonal().transpose() * 1e6 << std::endl;
    // std::cout << "Current pose cov: " << composition_cov.diagonal().transpose() * 1e6 << std::endl;
    // SharedNoiseModel noise_model = noiseModel::Gaussian::Covariance(composition_cov);

    std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
    oldGraph.emplace_shared<BetweenFactor<Pose3>>(X(p_index), X(c_index), prev_pose3.between(curr_pose3), noise_odom_between);
    graph_guard.unlock();
    loopIndexCheckedMap[c_index].push_back(p_index);
    std::cout << "[INFO] Added initial factor: " << p_index << " -> " << c_index << std::endl;

    // Add measurement data.
    {
        std::unique_lock<std::mutex> kf_guard(mKF);
        measurement.updated_pose = GtsamPose2Pose6D(curr_pose3);
        measurement.global_score = priorScore;
        oldMeasurements.push_back(measurement);
        keyMeasures.push_back(measurement);
    }
    std::cout << "[INFO] Updated measurements. Keyframes: " << keyMeasures.size()
              << ", Total measurements: " << oldMeasurements.size() << std::endl;

    // Update LIO state and previous pose.
    lioState2 = GetStateFromLIO2(0);
    keyLIOState2.push_back(lioState2);
    prevPose = curr_pose3;
}

//---------------------------------------------------------------------
// Optimize Pose Graph
//---------------------------------------------------------------------
void MSMapping::OptimizeGraph()
{
    oldValues.insert(X(curr_node_add_idx), predict_pose);
    bool use_isam = true;
    try
    {
        std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
        if (use_isam)
        {
            // std::cout << "[INFO] Updating ISAM2 with current graph..." << std::endl;
            isam->update(oldGraph, oldValues);
            isam->update();
            for (int i = 0; i < 5; ++i)
            {
                isam->update();
            }
            currentEstimate = isam->calculateEstimate();
            poseCovariance = isam->marginalCovariance(X(curr_node_add_idx));
        }
    }
    catch (const gtsam::IndeterminantLinearSystemException &e)
    {
        ROS_ERROR("GTSAM indeterminate linear system exception!");
        std::cerr << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("General exception caught during optimization!");
        std::cerr << e.what() << std::endl;
    }
    // Update previous states.
    prevPose = currentEstimate.at<Pose3>(X(curr_node_add_idx));
    prevState = NavState(prevPose, prevVel);
    // Clear old graph and values.
    oldGraph.resize(0);
    oldValues.clear();

    // Update measurements with optimized poses.
    std::unique_lock<std::mutex> kf_guard(mKF);
    for (int i = 0; i < oldMeasurements.size(); i++)
    {
        Pose3 pose = currentEstimate.at<Pose3>(X(i));
        Pose6D &p = oldMeasurements.at(i).updated_pose;
        p.x = pose.translation().x();
        p.y = pose.translation().y();
        p.z = pose.translation().z();
        p.roll = pose.rotation().roll();
        p.pitch = pose.rotation().pitch();
        p.yaw = pose.rotation().yaw();
        if (i >= old_node_idx)
        {
            Pose6D &p1 = keyMeasures.at(i - old_node_idx).updated_pose;
            p1.x = pose.translation().x();
            p1.y = pose.translation().y();
            p1.z = pose.translation().z();
            p1.roll = pose.rotation().roll();
            p1.pitch = pose.rotation().pitch();
            p1.yaw = pose.rotation().yaw();
        }
    }
    aLoopIsClosed = false;
    // aGlobalConstrained = false;
}

//---------------------------------------------------------------------
// Add Odometry Factor to Graph
//---------------------------------------------------------------------
void MSMapping::AddOdomFactorToOldGraph()
{
    Pose3 curr_lio_pose = Pose6dTogtsamPose3(measurement_curr.key_pose);
    Matrix6 pose_cov = lioState2.cov;
    Pose3 prev_lio_pose = Pose6dTogtsamPose3(keyMeasures.at(prev_node_idx).key_pose);
    rel_pose = prev_lio_pose.between(curr_lio_pose);
    predict_pose = prevPose.compose(rel_pose);

    std::unique_lock<std::mutex> graph_guard_1(mtxPosegraph);
    oldGraph.emplace_shared<BetweenFactor<Pose3>>(
        X(prev_node_add_idx), X(curr_node_add_idx), rel_pose, noise_odom_between);
    graph_guard_1.unlock();
    std::cout << "[INFO] Added odometry factor: " << prev_node_add_idx << " -> " << curr_node_add_idx << std::endl;
}

void MSMapping::AddMapPriorFactor()
{
    if (curr_node_idx < 1)
        return;

    TicToc tic_toc;
    useGlobalPrior = true;
    if (useGlobalPrior)
    {
        bool flag = GetGlobalICP(keyMeasures.at(curr_node_idx));
        if (flag)
        {
            Pose3 poseGlobal = Pose6dTogtsamPose3(keyMeasures.at(curr_node_idx).global_pose);
            std::unique_lock<std::mutex> graph_guard_3(mtxPosegraph);
            newFactors.emplace_shared<PriorFactor<Pose3>>(X(curr_node_add_idx), poseGlobal, priorMapPoseNoise);
            graph_guard_3.unlock();
            mapIndexContainer[curr_node_idx] = curr_node_idx;
        }
    }
}

void MSMapping::AddNoMotionFactor()
{
    if (curr_node_idx < 1)
        return;

    //    std::unique_lock<std::mutex> graph_guard1(mtxPosegraph);
    std::cout << BOLDRED << "add zero velocity factor" << std::endl;
    //    newFactors.emplace_shared<PriorFactor<Vector3 >>(
    //            V(curr_node_idx), Vector3::Zero(), zero_velocity_prior_noise_);
    //    graph_guard1.unlock();

    // add no motion factor
    std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
    oldGraph.emplace_shared<BetweenFactor<Pose3>>(X(prev_node_add_idx), X(curr_node_add_idx), Pose3::Identity(),
                                                  no_motion_prior_noise_);
    graph_guard.unlock();
}

bool MSMapping::SaveKeyframeRadius(Pose6D &dtf)
{
    if (keyMeasures.empty())
        return true;
    double delta_translation =
        sqrt(dtf.x * dtf.x + dtf.y * dtf.y + dtf.z * dtf.z);
    translationAccumulated += delta_translation;
    rotaionAccumulated +=
        (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.
    if (translationAccumulated > keyframeMeterGap)
    {
        isNowKeyFrame = true;
        translationAccumulated = 0.0;
        rotaionAccumulated = 0.0;
    }
    else
        isNowKeyFrame = false;
    return isNowKeyFrame;
}

void MSMapping::AddLoopFactor()
{
    if (loopIndexQueue.empty() || curr_node_idx < 5)
        return;

    double t3 = 0.0;
    std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
    for (int i = 0; i < loopIndexQueue.size(); ++i)
    {
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        //        SharedDiagonal noiseBetween = loopNoiseQueue[i];
        // TODO: WHY HERE BROKEN?
        SharedNoiseModel noiseBetween = loopGaussianNoiseQueue[i];
        oldGraph.emplace_shared<BetweenFactor<Pose3>>(X(indexFrom), X(indexTo), poseBetween, noiseBetween);
        // record time cost
        t3_all += loopTime[i];
        t3 += loopTime[i];
    }
    graph_guard.unlock();

    std::unique_lock<std::mutex> loop_guard(mtxLoopContainer);
    loopIndexQueue.clear();
    loopPoseQueue.clear();
    //    loopNoiseQueue.clear();
    loopGaussianNoiseQueue.clear();
    loopTime.clear();
    loop_guard.unlock();

    aLoopIsClosed = true;
}

void MSMapping::AddGPSFactor()
{
    if (keyMeasures.back().gps.header.seq == -1)
        return;
    nav_msgs::Odometry currGPSOdometry = keyMeasures.back().gps;
    Pose6D curr_pose = keyMeasures.back().updated_pose;

    // GPS too noisy, skip
    float gps_x = currGPSOdometry.pose.pose.position.x;
    float gps_y = currGPSOdometry.pose.pose.position.y;
    float gps_z = currGPSOdometry.pose.pose.position.z;
    float noise_x = currGPSOdometry.pose.covariance[0];
    float noise_y = currGPSOdometry.pose.covariance[7];
    float noise_z = currGPSOdometry.pose.covariance[14];
    float status = currGPSOdometry.pose.covariance[4];

    // make sure zhe gps data is stable encough
    if (abs(noise_x) > gpsCovThreshold && abs(noise_y) > gpsCovThreshold)
        return;
    if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
        return;

    // ROS_WARN(" gps lidar off time: %f ", off_time);
    //        std::cout << "currlidar pose: " << transformTobeMapped[3] << ", " <<
    //        transformTobeMapped[4] << ", "
    //                  << transformTobeMapped[5] << std::endl;
    //        std::cout << "currGPS position: " << gps_x << ", " << gps_y << ", "
    //        << gps_z << std::endl; std::cout << "currGPS cov: " <<
    //        thisGPS.pose.covariance[0] << ", " << thisGPS.pose.covariance[7]
    //                  << ", " << thisGPS.pose.covariance[14] << std::endl;

    // Add GPS every a few meters
    PointT curGPSPoint;
    Pose6D gpsPoint;
    gpsPoint.x = curGPSPoint.x = gps_x;
    gpsPoint.y = curGPSPoint.y = gps_y;
    gpsPoint.z = curGPSPoint.z = gps_z;

    if (pointDistance(curGPSPoint, lastGPSPoint) < GPS_SKIP_NUMBER)
        return;
    lastGPSPoint = curGPSPoint;

    ROS_INFO("curr gps pose: %f, %f , %f", gps_x, gps_y, gps_z);
    ROS_INFO("curr gps cov: %f, %f , %f", noise_x, noise_y, noise_z);
    // LOG(INFO) << "GPS Z OFFSET:" << abs(gps_z - curr_pose.z);

    noiseModel::Diagonal::shared_ptr gpsNoiseModel =
        noiseModel::Diagonal::Variances(
            (gtsam::Vector(3) << currGPSOdometry.pose.covariance[0],
             currGPSOdometry.pose.covariance[7],
             currGPSOdometry.pose.covariance[14])
                .finished());

    gtsam::GPSFactor gps_factor(
        X(curr_node_idx), gtsam::Point3(gps_x, gps_y, gps_z), gpsNoiseModel);
    keyframeGPSfactor.push_back(gps_factor);
    keyframeGPS.push_back(gpsPoint);

    // only a trick!
    // we need to accumulate some accurate gps points to initialize the transform
    // between gps coordinate system and LIO coordinate system
    // and then we can add gps points one by one into the pose graph
    // or the whole pose graph will crashed if giving some respectively bad gps
    // points at first.
    if (keyframeGPSfactor.size() < accumulatedFactorSize)
    {
        ROS_INFO("Accumulated gps factor: %d", keyframeGPSfactor.size());
        return;
    }

    if (!gpsTransfromInit)
    {
        ROS_INFO("Initialize GNSS transform!");
        std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
        for (int i = 0; i < keyframeGPSfactor.size(); ++i)
        {
            newFactors.emplace_shared<gtsam::GPSFactor>(keyframeGPSfactor.at(i));
            gpsIndexContainer[keyframeGPSfactor.at(i).key()] = i;
        }
        graph_guard.unlock();
        gpsTransfromInit = true;
    }
    else
    {
        // After the coordinate systems are aligned, in theory, the GPS z and the z
        // estimated by the current LIO system should not be too different.
        // Otherwise, there is a problem with the quality of the secondary GPS
        // point.
        if (abs(gps_z - curr_pose.z) > 30.0 || abs(gps_x - curr_pose.x) > 30)
        {
            // ROS_WARN("Too large GNSS z noise %f", noise_z);
            gtsam::Vector Vector3(3);
            Vector3 << std::max(noise_x, 10000000.0f), std::max(noise_y, 10000000.0f),
                std::max(noise_z, 100000000.0f);
            //            gps_noise = noiseModel::Diagonal::Variances(Vector3);
            //            gps_factor = gtsam::GPSFactor(curr_node_idx,
            //            gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
        }

        std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
        newFactors.emplace_shared<GPSFactor>(gps_factor);
        graph_guard.unlock();
        gpsIndexContainer[curr_node_idx] = keyframeGPS.size() - 1;
        // LOG(WARNING) << " ADD GPS OPTIMIZED NODE: " << keyframeGPS.size();
    }
    aLoopIsClosed = true;
}

bool MSMapping::ZUPTDetector()
{
    if (curr_node_idx < 1)
        return false;

    Pose6D df = diffTransformation(odom_pose_prev, odom_pose_curr);
    double aver_translation = sqrt(df.x * df.x + df.y * df.y + df.z * df.z) / 3;
    double aver_rotation = sqrt(df.roll + df.pitch + df.yaw) / 3;

    if (0)
    {
        std::cout << "average lio translation and angle: " << aver_translation
                  << ", " << aver_rotation << std::endl;
    }

    auto imu_queue = keyMeasures.at(curr_node_idx).imu_deque;

    if (imu_queue.empty())
        return false;

    Eigen::Vector3d gyro_delta_average, acc_deltea_average;
    Eigen::Vector3d acc_average, gyro_average;
    Eigen::Vector3d acc_diff_max, gyro_diff_max;
    Eigen::Vector3d acc_diff_min, gyro_diff_min;

    acc_diff_max << imu_queue.at(0).linear_acceleration.x,
        imu_queue.at(0).linear_acceleration.y,
        imu_queue.at(0).linear_acceleration.z;
    acc_diff_min = acc_diff_max;

    gyro_diff_max << imu_queue.at(0).angular_velocity.x,
        imu_queue.at(0).angular_velocity.y, imu_queue.at(0).angular_velocity.z;
    gyro_diff_min = gyro_diff_max;

    for (auto it_imu = imu_queue.begin(); it_imu < (imu_queue.end() - 1);
         it_imu++)
    {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        Eigen::Vector3d gyro, acc;
        gyro << head.angular_velocity.x, head.angular_velocity.y,
            head.angular_velocity.z;
        acc << head.linear_acceleration.x, head.linear_acceleration.y,
            head.linear_acceleration.z;
        acc_average += acc;
        gyro_average += gyro;

        acc_diff_max = max(acc_diff_max, acc);
        gyro_diff_max = max(gyro_diff_max, gyro);
        acc_diff_min = min(acc_diff_min, acc);
        gyro_diff_min = min(gyro_diff_min, gyro);

        Eigen::Vector3d gyro_delta, acc_delta;
        gyro_delta << abs(tail.angular_velocity.x - head.angular_velocity.x),
            abs(tail.angular_velocity.y - head.angular_velocity.y),
            abs(tail.angular_velocity.z - head.angular_velocity.z);
        gyro_delta_average += gyro_delta;

        acc_delta << abs(head.linear_acceleration.x - tail.linear_acceleration.x),
            abs(head.linear_acceleration.y - tail.linear_acceleration.y),
            abs(head.linear_acceleration.z - tail.linear_acceleration.z);
        acc_deltea_average += acc_delta;
    }
    gyro_delta_average = gyro_delta_average / (imu_queue.size() - 1);
    acc_deltea_average = acc_deltea_average / (imu_queue.size() - 1);

    acc_average /= (imu_queue.size() - 1);
    gyro_average /= (imu_queue.size() - 1);

    double accel_norm = std::sqrt(
        std::pow(acc_average.x(), 2) +
        std::pow(acc_average.y(), 2) +
        std::pow(acc_average.z(), 2));

    double ang_vel_norm = std::sqrt(
        std::pow(gyro_average.x(), 2) +
        std::pow(gyro_average.y(), 2) +
        std::pow(gyro_average.z(), 2));

    //    Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr);
    //    double lidar_trans = std::sqrt(
    //            std::pow(dtf.x, 2) +
    //            std::pow(dtf.y, 2) +
    //            std::pow(dtf.z, 2)
    //    );
    //    double lidar_rotate = std::sqrt(
    //            std::pow(dtf.roll, 2) +
    //            std::pow(dtf.pitch, 2) +
    //            std::pow(dtf.yaw, 2)
    //    );

    if (0)
    {
        std::cout << "average acc gyro: " << accel_norm << ", "
                  << ang_vel_norm << std::endl;
        std::cout << "acc_min max: " << acc_diff_min.transpose() << ", "
                  << acc_diff_max.transpose() << std::endl;
        std::cout << "gyro min max: " << gyro_diff_min.transpose() << ", "
                  << gyro_diff_max.transpose() << std::endl;
        std::cout << "delta acc : " << acc_deltea_average.transpose() << ", "
                  << gyro_delta_average.transpose() << std::endl;
    }

    //    if (ang_vel_norm > imu_ang_vel_threshold) {
    //        return false;
    //    }
    //    if (accel_norm > imu_accel_threshold) {
    //        return false;
    //    }

    if ((acc_diff_max.x() > 0.01) || (acc_diff_max.y() > 0.01) ||
        (acc_diff_max.z() > 0.01) || (gyro_diff_max.x() > 0.025) ||
        (gyro_diff_max.y() > 0.025) || (gyro_diff_max.z() > 0.025))
    {
        // return NONSTATIONARY;
    }
    else
    {
        // return STATIONARY;
    }
    //    }

    double acc_thro = 0.003, anguler_thro = 0.003;
    //  bool zupt_flag =
    //      (angvel_avr[0] < anguler_thro && angvel_avr[1] < anguler_thro &&
    //       angvel_avr[2] < anguler_thro) ||
    //      (acc_avr[0] < acc_thro && acc_avr[1] < acc_thro && acc_avr[2] <
    //      acc_thro);
    // }

    bool zupt_flag = (aver_translation < 0.01 && aver_rotation < 0.015);

    return zupt_flag;
}

bool MSMapping::SyncGPS(std::deque<nav_msgs::Odometry> &gpsBuf,
                        nav_msgs::Odometry &alignedGps, double timestamp,
                        double eps_cam)
{
    if (gpsQueue.empty())
    {
        return false;
    }
    bool gpsFound = false;
    {
        std::unique_lock<std::mutex> guard(mutexLock);
        while (!gpsQueue.empty())
        {
            double gpsTime = gpsQueue.front().header.stamp.toSec();
            if (gpsTime < timestamp - eps_cam)
            {
                // The GPS data is too old, discard it
                gpsQueue.pop_front();
            }
            else if (gpsTime > timestamp + eps_cam)
            {
                // The GPS data is too new, stop processing
                break;
            }
            else
            {
                // Found a GPS data close to the desired timestamp
                gpsFound = true;
                alignedGps = gpsQueue.front();
                gpsQueue.pop_front();
                break;
            }
        }
    } // mutex is automatically unlocked here
    return gpsFound;
}

bool MSMapping::SyncData(Measurement &measurement)
{
    sensor_msgs::PointCloud2 raw_cloud_msg;
    nav_msgs::Odometry thisOdom;
    double odom_time, cloud_time;

    std::unique_lock<std::mutex> data_guard(mutexLock, std::defer_lock);
    if (data_guard.try_lock())
    {
        if (odometryBuf.empty() || fullResBuf.empty())
        {
            return false;
        }
        odom_time = odometryBuf.front()->header.stamp.toSec();
        cloud_time = fullResBuf.front()->header.stamp.toSec();
        // Discard old odometry data
        while (!odometryBuf.empty() && (odom_time < cloud_time - 0.05))
        {
            ROS_WARN("odometry discarded: %f", odom_time - cloud_time);
            odometryBuf.pop();
            odom_time = odometryBuf.empty() ? std::numeric_limits<double>::max()
                                            : odometryBuf.front()->header.stamp.toSec();
        }
        // Discard old point cloud data
        while (!fullResBuf.empty() && (cloud_time < odom_time - 0.05))
        {
            ROS_WARN("pointCloud discarded: %f", cloud_time - odom_time);
            fullResBuf.pop();
            cloud_time = fullResBuf.empty() ? std::numeric_limits<double>::max()
                                            : fullResBuf.front()->header.stamp.toSec();
        }
        if (fullResBuf.empty() || odometryBuf.empty())
        {
            return false;
        }
        raw_cloud_msg = *(fullResBuf.front());
        thisOdom = *odometryBuf.front();
        fullResBuf.pop();
        odometryBuf.pop();

        data_guard.unlock();
    }
    else
    {
        // 无法获取锁，可能因为其他线程正在使用资源
        return false;
    }

    /*    {
            std::unique_lock<std::mutex> data_guard(mutexLock);
            odom_time = odometryBuf.front()->header.stamp.toSec();
            cloud_time = fullResBuf.front()->header.stamp.toSec();
            // Discard old odometry data
            while (!odometryBuf.empty() && (odom_time < cloud_time - 0.05)) {
                ROS_WARN("odometry discarded: %f", odom_time - cloud_time);
                odometryBuf.pop();
                odom_time = odometryBuf.empty() ? std::numeric_limits<double>::max()
                                                : odometryBuf.front()->header.stamp.toSec();
            }
            // Discard old point cloud data
            while (!fullResBuf.empty() && (cloud_time < odom_time - 0.05)) {
                ROS_WARN("pointCloud discarded: %f", cloud_time - odom_time);
                fullResBuf.pop();
                cloud_time = fullResBuf.empty() ? std::numeric_limits<double>::max()
                                                : fullResBuf.front()->header.stamp.toSec();
            }
            if (fullResBuf.empty() || odometryBuf.empty()) {
                return false;
            }
            raw_cloud_msg = *(fullResBuf.front());
            thisOdom = *odometryBuf.front();
            fullResBuf.pop();
            odometryBuf.pop();
        }*/

    // Process lidar data...
    // 处理 lidar 数据时重复利用已分配的资源
    pcl::PointCloud<PointT>::Ptr thisKeyFrame(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(raw_cloud_msg, *thisKeyFrame);
    pcl::PointCloud<PointT>::Ptr thisKeyFrameTemp(new pcl::PointCloud<PointT>());
    thisKeyFrameTemp = cloud_process_.RemoveRangeCloud(thisKeyFrame, Eigen::Vector3i(1, 1, 1), Eigen::Vector3d(60, 60, 60), ">");

    if (useRawCloud)
    {
        // pcl::transformPointCloud(*thisKeyFrameTemp, *thisKeyFrameTemp,
        // T_body_lidar.cast<float>());
        measurement.lidar = thisKeyFrameTemp;
    }
    else
    {
        pcl::PointCloud<PointT>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointT>());
        downSizeFilterScan.setInputCloud(thisKeyFrameTemp);
        downSizeFilterScan.filter(*thisKeyFrameDS);
        // pcl::transformPointCloud(*thisKeyFrameDS, *thisKeyFrameDS,
        // T_body_lidar.cast<float>());
        measurement.lidar = thisKeyFrameDS;
    }
    measurement.lidar_time = cloud_time;
    measurement.odom_time = odom_time;
    measurement.odom = thisOdom;
    // std::cout << "SYNC ODOM COV: " << measurement.odom.pose.covariance.at(0) << std::endl;
    //  Sync IMU data
    measurement.imu_deque.clear();
    {
        std::unique_lock<std::mutex> imu_guard(imuLock);
        while (!imuQueue.empty() && (imuQueue.front().header.stamp.toSec() < cloud_time))
        {
            measurement.imu_deque.push_back(imuQueue.front());
            imuQueue.pop_front();
        }
    }
    // Process and align IMU pose...
    // Align GPS data if used
    bool has_GPS = false;
    nav_msgs::Odometry currGPSodom;
    if (useGPS)
    {
        has_GPS = SyncGPS(gpsQueue, currGPSodom, odom_time, 1.0 / gpsFrequence);
        //        if (currGPSodom.pose.covariance.at(14) != 1.0) {
        //            has_GPS = false;
        //        }
    }
    if (useGPS && has_GPS)
    {
        measurement.gps = currGPSodom;
        Pose3 gps_pose3 = Pose6dTogtsamPose3(getOdom(currGPSodom));
        // std::cout << "HAS GPS: " << gps_pose3.translation().transpose() << std::endl;
    }
    measurement.has_gps = has_GPS;
    return true;
}

void MSMapping::PubPath()
{
    // pub odom and path
    nav_msgs::Odometry odomAftPGO, localizationOdom;
    nav_msgs::Path pathAftPGO, pathLocalization, pathLIO, pathIMU, pathNewPGO;
    pathAftPGO.header.frame_id = odom_link;
    pathNewPGO.header.frame_id = odom_link;
    pathLIO.header.frame_id = odom_link;
    pathIMU.header.frame_id = odom_link;
    odomAftPGO.header.frame_id = odom_link;
    localizationOdom.header.frame_id = odom_link;
    pathLocalization.header.frame_id = odom_link;
    odomAftPGO.child_frame_id = "/aft_pgo";

    for (int node_idx = 0; node_idx < oldMeasurements.size(); node_idx++)
    {
        const Pose6D pose_est = oldMeasurements.at(node_idx).updated_pose;
        //        const Pose6D pose_loc = oldMeasurements.at(node_idx).global_pose;
        //        const Pose6D odom_pose = oldMeasurements.at(node_idx).key_pose;
        odomAftPGO.header.stamp =
            ros::Time().fromSec(oldMeasurements.at(node_idx).odom_time);
        odomAftPGO.pose.pose.position.x = pose_est.x;
        odomAftPGO.pose.pose.position.y = pose_est.y;
        odomAftPGO.pose.pose.position.z = pose_est.z;
        odomAftPGO.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
            pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGO.pose.covariance.at(0) = isDegenerate;

        geometry_msgs::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGO.header;
        poseStampAftPGO.pose = odomAftPGO.pose.pose;
        //        Pose6D odom_pose_trans = odom_pose;
        //        geometry_msgs::PoseStamped poseStampLIO;
        //        poseStampLIO.header = odomAftPGO.header;
        //        poseStampLIO.pose.position.x = odom_pose_trans.x;
        //        poseStampLIO.pose.position.y = odom_pose_trans.y;
        //        poseStampLIO.pose.position.z = odom_pose_trans.z;
        //        poseStampLIO.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
        //                odom_pose_trans.roll, odom_pose_trans.pitch, odom_pose_trans.yaw);
        //        pathLIO.header.stamp = odomAftPGO.header.stamp;
        //        pathLIO.poses.push_back(poseStampLIO);

        if (node_idx >= old_node_idx)
        {
            pathNewPGO.header.stamp = odomAftPGO.header.stamp;
            pathNewPGO.poses.push_back(poseStampAftPGO);
        }
        else
        {
            pathAftPGO.header.stamp = odomAftPGO.header.stamp;
            pathAftPGO.poses.push_back(poseStampAftPGO);
        }

        // imu path
        /*        if (oldMeasurements.at(node_idx).has_imu) {
                    const nav_msgs::Odometry imu_optimized_odom = oldMeasurements.at(node_idx).imu_odom;

                    geometry_msgs::PoseStamped poseStampIMU;
                    poseStampIMU.header = imu_optimized_odom.header;
                    poseStampIMU.pose.position.x = imu_optimized_odom.pose.pose.position.x;
                    poseStampIMU.pose.position.y = imu_optimized_odom.pose.pose.position.y;
                    poseStampIMU.pose.position.z = imu_optimized_odom.pose.pose.position.z;
                    poseStampIMU.pose.orientation = imu_optimized_odom.pose.pose.orientation;
                    pathIMU.header.stamp = imu_optimized_odom.header.stamp;
                    pathIMU.poses.push_back(poseStampIMU);
        //            pathAftPGO.header.stamp = odomAftPGO.header.stamp;
        //            pathAftPGO.poses.push_back(poseStampAftPGO);
                }*/

        /* if (oldMeasurements.at(node_idx).global_score != 0) {
             localizationOdom.header.stamp =
                     ros::Time().fromSec(oldMeasurements.at(node_idx).odom_time);
             localizationOdom.pose.pose.position.x = pose_loc.x;
             localizationOdom.pose.pose.position.y = pose_loc.y;
             localizationOdom.pose.pose.position.z = pose_loc.z;
             localizationOdom.pose.pose.orientation =
                     tf::createQuaternionMsgFromRollPitchYaw(pose_loc.roll, pose_loc.pitch,
                                                             pose_loc.yaw);
             localizationOdom.pose.covariance.at(0) = pose_loc.cov(0, 0);
             localizationOdom.pose.covariance.at(7) = pose_loc.cov(0, 1);
             localizationOdom.pose.covariance.at(14) = pose_loc.cov(0, 2);
             localizationOdom.pose.covariance.at(21) = pose_loc.cov(0, 3);
             localizationOdom.pose.covariance.at(28) = pose_loc.cov(0, 4);
             localizationOdom.pose.covariance.at(35) = pose_loc.cov(0, 5);

             geometry_msgs::PoseStamped poseStampLoc;
             poseStampLoc.header = localizationOdom.header;
             poseStampLoc.pose = localizationOdom.pose.pose;
             pathLocalization.header.stamp = localizationOdom.header.stamp;
             pathLocalization.poses.push_back(poseStampLoc);
         }*/
    }
    //    odomAftPGO.pose.covariance.at(0) = eigen_values[0];
    //    odomAftPGO.pose.covariance.at(7) = eigen_values[1];
    //    odomAftPGO.pose.covariance.at(14) = eigen_values[2];
    //    odomAftPGO.pose.covariance.at(0) = poseCovariance(3, 3) * 1e6;
    //    odomAftPGO.pose.covariance.at(7) = poseCovariance(4, 4) * 1e6;
    //    odomAftPGO.pose.covariance.at(14) = poseCovariance(5, 5) * 1e6;

    // 调整poseCovariance的列顺序
    Eigen::MatrixXd adjustedCovariance(6, 6);
    adjustedCovariance.block<3, 3>(0, 0) = poseCovariance.block<3, 3>(3, 3); // 平移部分
    adjustedCovariance.block<3, 3>(0, 3) = poseCovariance.block<3, 3>(3, 0); // 平移与旋转之间的协方差
    adjustedCovariance.block<3, 3>(3, 0) = poseCovariance.block<3, 3>(0, 3); // 旋转与平移之间的协方差
    adjustedCovariance.block<3, 3>(3, 3) = poseCovariance.block<3, 3>(0, 0); // 旋转部分
                                                                             // 将调整后的矩阵赋值给odomAftPGO.pose.covariance
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            odomAftPGO.pose.covariance.at(i * 6 + j) = adjustedCovariance(i, j);
        }
    }

    pubOdomAftPGO.publish(odomAftPGO);
    // pubOdomAftGlobal.publish(localizationOdom);
    pubPathAftPGO.publish(pathAftPGO);
    pubPathNewpgo.publish(pathNewPGO);
    // pubLocalizationPath.publish(pathLocalization);
    // pubPathLIO.publish(pathLIO);

    // publish latest imu path
    //    while (!pathIMU.poses.empty() &&
    //           pathIMU.poses.front().header.stamp.toSec() < keyMeasures.at(curr_node_idx).lidar_time - 3.0)
    //        pathIMU.poses.erase(pathIMU.poses.begin());

    //    if (pubPathLIO.getNumSubscribers() != 0) {
    //        pathIMU.header.stamp = ros::Time().fromSec(oldMeasurements.at(curr_node_add_idx).lidar_time);
    //        pathIMU.header.frame_id = odom_link;
    //        pubPathIMU.publish(pathIMU);
    //    }

    // publish current cloud
    pcl::PointCloud<PointT>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointT>);
    *transform_cloud_ptr = *TransformPointCloud(oldMeasurements[curr_node_add_idx].lidar,
                                                oldMeasurements[curr_node_add_idx].updated_pose);
    publishCloud(pubLaserCloudSurround, transform_cloud_ptr,
                 ros::Time().fromSec(oldMeasurements.at(curr_node_add_idx).odom_time),
                 odom_link);

    // send transform
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftPGO.pose.pose.position.x,
                                    odomAftPGO.pose.pose.position.y,
                                    odomAftPGO.pose.pose.position.z));
    q.setW(odomAftPGO.pose.pose.orientation.w);
    q.setX(odomAftPGO.pose.pose.orientation.x);
    q.setY(odomAftPGO.pose.pose.orientation.y);
    q.setZ(odomAftPGO.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftPGO.header.stamp,
                                          odom_link, "/aft_pgo"));
}

void MSMapping::PubMap()
{
    static pcl::PointCloud<PointT>::Ptr temp_cloud_ptr(new pcl::PointCloud<PointT>());

    if (keyMeasures.size() < 5)
        return;
    laserCloudMapPGO->clear();
    temp_cloud_ptr->clear();
    {
        SKIP_FRAMES = 10;
        std::unique_lock<std::mutex> kf_guard(mKF);
        for (int node_idx = 0; node_idx < keyMeasures.size(); node_idx++)
        {
            if (node_idx % SKIP_FRAMES == 0)
            {
                temp_cloud_ptr->clear();
                Measurement measurement_temp = keyMeasures.at(node_idx);
                Eigen::Matrix4f transformation = Pose6D2Matrix(measurement_temp.updated_pose).cast<float>();
                transformPointCloud(*measurement_temp.lidar, *temp_cloud_ptr, transformation);
                *laserCloudMapPGO += *temp_cloud_ptr;
            }
        }
    }
    if (!laserCloudMapPGO->empty())
    {
        downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
        downSizeFilterMapPGO.filter(*laserCloudMapPGO);
    }
    publishCloud(pubMapAftPGO, laserCloudMapPGO, ros::Time::now(), odom_link);
}

void MSMapping::LoopDetection()
{
    ros::Rate rate(loopClosureFrequency);
    while (!stopRequested.load() && ros::ok())
    {
        ros::spinOnce();
        {
            PerformRSLoopClosure();
        }
        rate.sleep();
    }
}

void MSMapping::VisualizationThread()
{
    float vizmapFrequency = 0.1; // 0.1 means run onces every 10s
    ros::Rate rate(vizmapFrequency);
    while (!stopRequested.load() && ros::ok())
    {
        rate.sleep();
        PubMap();
        if (useLoopCloser)
        {
            VisualizeLoopConstrains(loopIndexCheckedMap, pubLoopConstraintEdge, 0);
        }
    }
}

void MSMapping::SaveData()
{
    if (keyMeasures.empty())
    {
        ROS_INFO("NO ENCOUGH POSE!");
        return;
    }

    int size = oldMeasurements.size();
    int new_size = keyMeasures.size();
    int diff_size = size - new_size;
    std::vector<Eigen::Vector3d> lla_vec;
    std::vector<nav_msgs::Odometry> odomUpdated_vec;
    std::vector<double> keyframeTimes;
    std::vector<nav_msgs::Odometry> keyframePosesOdom;
    std::vector<pcl::PointCloud<PointT>::Ptr> keyframeLaserClouds;
    std::vector<sensor_msgs::PointCloud2> keyframeLaserCloudsMsg;
    std::vector<Vector7> pose_vec;
    std::vector<Eigen::Isometry3d> global_pose_vec;
    std::vector<Vector7> global_pose_vec_traj;
    std::vector<Vector7> imu_pose_vec_traj;

    std::string dir_path = saveDirectory + sequence + "/";
    std::ofstream logfile_time(dir_path + "data_time.txt", std::ios_base::out);
    logfile_time.precision(12);
    std::ofstream outFile(dir_path + "pose_graph_3d_result.txt");
    outFile.precision(12);

    std::cout << "=========================================================== Save Data Begin =========================================================== " << std::endl;
    std::cout << "write time file path: " << saveDirectory + "data_time.txt" << std::endl;
    std::cout << "New session node: " << diff_size << " " << old_node_idx << " " << new_size << std::endl;

    std::unique_lock<std::mutex> kf_guard(mKF);
    Values result = isam->calculateEstimate();
    Marginals marginals(isam->getFactorsUnsafe(), result, Marginals::Factorization::CHOLESKY);
    for (int i = 0; i < size; ++i)
    {
        Measurement mea = oldMeasurements.at(i);
        Pose6D p = GtsamPose2Pose6D(result.at<gtsam::Pose3>(X(i)));
        Pose6D p2 = mea.key_pose;
        double time = mea.odom_time;
        double lidar_time = mea.lidar_time;
        keyframeTimes.push_back(time);
        keyframePosesOdom.push_back(mea.odom);
        keyframeLaserClouds.push_back(mea.lidar);

        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = ros::Time().fromSec(time);
        laserOdometryROS.header.frame_id = "map";
        laserOdometryROS.child_frame_id = "lidar";
        laserOdometryROS.pose.pose.position.x = p.x;
        laserOdometryROS.pose.pose.position.y = p.y;
        laserOdometryROS.pose.pose.position.z = p.z;
        laserOdometryROS.pose.pose.orientation =
            tf::createQuaternionMsgFromRollPitchYaw(p.roll, p.pitch, p.yaw);
        odomUpdated_vec.push_back(laserOdometryROS);

        pose_vec.push_back((Vector7() << p.x, p.y, p.z,
                            laserOdometryROS.pose.pose.orientation.x,
                            laserOdometryROS.pose.pose.orientation.y,
                            laserOdometryROS.pose.pose.orientation.z,
                            laserOdometryROS.pose.pose.orientation.w)
                               .finished());
        if (i >= diff_size)
        {
            if (result.exists(X(i)))
            { // 检查键是否存在
                auto pose = result.at<gtsam::Pose3>(X(i));
                // 保存姿态
                gtsam::Point3 translation = pose.translation();
                gtsam::Rot3 rotation = pose.rotation();
                gtsam::Quaternion quaternion = rotation.toQuaternion();
                outFile << time << " "
                        << translation.x() << " " << translation.y() << " " << translation.z() << " "
                        << quaternion.w() << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z();
                // 保存协方差矩阵
                auto covariance = marginals.marginalCovariance(X(i));
                for (int i = 0; i < covariance.rows(); ++i)
                {
                    for (int j = 0; j < covariance.cols(); ++j)
                    {
                        outFile << " " << covariance(i, j);
                    }
                }
                outFile << std::endl;

                Vector10 time_vec = mea.time_data;
                logfile_time << time << " ";
                for (int j = 0; j < time_vec.size(); ++j)
                {
                    logfile_time << time_vec(j);
                    if (j != time_vec.size() - 1)
                    {
                        logfile_time << " ";
                    }
                }
                logfile_time << std::endl;
            }
        }
    }
    kf_guard.unlock();
    outFile.close();
    logfile_time.close();

    // 保存结果到文件
    std::cout << BOLDRED << "ISAM size: " << result.size() << " "
              << keyframePosesOdom.size() << " " << odomUpdated_vec.size() << " "
              << keyframeLaserClouds.size() << " " << pose_vec.size() << " "
              << global_pose_vec.size() << " " << keyTimeData.size() << " "
              << keyICPData.size() << std::endl;

    dataSaverPtr->saveTimes(keyframeTimes);
    dataSaverPtr->saveOdometryVerticesTUM(keyframePosesOdom);
    dataSaverPtr->saveOptimizedVerticesTUM(pose_vec, "optimized_poses_tum.txt");

    std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
    dataSaverPtr->saveGraphGtsam(oldGraph, isam, currentEstimate);
    graph_guard.unlock();

    //    if (saveResultBag) {
    //        dataSaverPtr->saveResultBag(keyframePosesOdom, odomUpdated_vec,
    //                                    keyframeLaserCloudsMsg);
    // dataSaverPtr->saveResultBag(odomUpdated, keyframeLaserClouds);
    //    }
    dataSaverPtr->savePointCloudMap(odomUpdated_vec, keyframeLaserClouds, diff_size);
    std::cout << "=========================================================== Save Data End =========================================================== " << std::endl;
}

void MSMapping::LidarCallback(
    const sensor_msgs::PointCloud2ConstPtr &pc_msg_ptr)
{
    std::unique_lock<std::mutex> guard(mutexLock);
    fullResBuf.push(pc_msg_ptr);
    guard.unlock();
}

void MSMapping::OdometryCallback(
    const nav_msgs::OdometryConstPtr &odom_msg_ptr)
{
    std::unique_lock<std::mutex> guard(mutexLock);
    odometryBuf.push(odom_msg_ptr);
    guard.unlock();
}

void MSMapping::OdometryIMUCallback(
    const nav_msgs::OdometryConstPtr &odom_msg_ptr)
{
    std::unique_lock<std::mutex> guard(mutexLock);
    odometryIMUBuf.push(odom_msg_ptr);
    guard.unlock();
}

void MSMapping::ImuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg)
{
    std::unique_lock<std::mutex> imu_guard(imuLock);
    imuQueue.push_back(*imuMsg);
    imu_guard.unlock();
}

void MSMapping::InitialCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg_ptr)
{
    initialPose = GeoposeToMatrix4d(*pose_msg_ptr);
    std::cout << BOLDBLUE << "Get Rviz Pose estimation: " << initialPose.matrix() << std::endl;
    poseReceived = true;
}

void MSMapping::GpsCallback(const nav_msgs::OdometryConstPtr &msg_ptr)
{
    std::unique_lock<std::mutex> guard(mutexLock);
    gpsQueue.push_back(*msg_ptr);
    guard.unlock();
}

void MSMapping::LoopInfoHandler(
    const std_msgs::Float64MultiArray::ConstPtr &loopMsg)
{
    if (loopMsg->data.size() != 2)
    {
        return;
    }
}

bool MSMapping::SaveMap(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res)
{
    ROS_WARN("SAVE MAP AND G2O..");

    SaveData();

    return true;
}

void MSMapping::PerformRSLoopClosure(void)
{
    const int minKeyFrames = 1;
    TicToc ticToc;
    if (keyMeasures.size() < minKeyFrames)
        return;
    int loopKeyCur = oldMeasurements.size() - 1;
    int loopKeyPre = -1;
    if (!DetectLoopClosureDistance(&loopKeyCur, &loopKeyPre))
        return;
    Pose6D cur_pose = oldMeasurements.at(loopKeyCur).updated_pose;
    Pose6D pre_pose = oldMeasurements.at(loopKeyPre).updated_pose;
    double loopScore = std::numeric_limits<double>::max();
    Eigen::Matrix4d trans_cur2pre = Pose6D2Matrix(cur_pose);
    pcl::PointCloud<PointT>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointT>());

    bool flag = false;
    {
        std::unique_lock<std::mutex> icp_guard(mtxICP);
        flag = cloud_process_.DoICPVirtualRelative3(oldMeasurements, targetKeyframeCloud, mapDirectory + "key_point_frame/",
                                                    loopKeyPre, loopKeyCur, loopScore, 0, trans_cur2pre);
        icp_guard.unlock();
    }
    if (flag)
    {
        if (!targetKeyframeCloud->empty())
        {
            publishCloud(pubLoopSubmapLocal, targetKeyframeCloud, ros::Time::now(), odom_link);
        }
        Pose3 pre_pose3 = Pose6dTogtsamPose3(pre_pose);
        Pose3 cur_pose3 = Matrix4dToPose3(trans_cur2pre);
        auto LOOPGaussianNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
        {
            std::unique_lock<std::mutex> loop_guard(mtxLoopContainer);
            loopIndexQueue.push_back(std::make_pair(loopKeyCur, loopKeyPre));
            loopPoseQueue.push_back(cur_pose3.between(pre_pose3));
            loopGaussianNoiseQueue.push_back(LOOPGaussianNoise);
            loop_guard.unlock();
        }
        loopIndexCheckedMap[loopKeyCur].push_back(loopKeyPre);
        std::cout << BOLDRED << "ICP Loop detected! " << loopKeyCur << " and " << loopKeyPre << ", " << loopScore << std::endl;
        loopTime.push_back(ticToc.toc());
    }
}

bool MSMapping::FilterLoopPairs(int loopKeyCur, int loopKeyPre)
{
    if (loopKeyCur == loopKeyPre || loopKeyPre == -1)
        return false;

    // short time
    if (abs(oldMeasurements.at(loopKeyCur).odom_time -
                abs(oldMeasurements.at(loopKeyPre).odom_time) <
            historyKeyframeSearchTimeDiff))
        return false;

    // the loop pairs exits in the icp detect container
    auto it = loopIndexCheckedMap.find(loopKeyCur);
    //    if (it != loopIndexCheckedMap.end()) {
    //        int loop_pre_candidate = it->second;
    //        // too closed loop pairs are useless
    //        if (abs(loop_pre_candidate - loopKeyPre) < filterNodeNum) return false;
    //    }
    if (it != loopIndexCheckedMap.end())
    {
        // 遍历所有与loopKeyCur关联的loop_pre_candidate值
        for (int loop_pre_candidate : it->second)
        {
            // 对每个loop_pre_candidate执行检查
            if (abs(loop_pre_candidate - loopKeyPre) < filterNodeNum)
                return false;
        }
    }

    auto it_sc = loopIndexSCcontainer.find(loopKeyCur);
    if (it_sc != loopIndexSCcontainer.end())
    {
        int loop_pre_candidate = it_sc->second;
        if (abs(loop_pre_candidate - loopKeyPre) < filterNodeNum)
            return false;
    }

    // the loop pairs exits in the intensity detect container
    auto it_v = loopVisualIndexCheckedMap.find(loopKeyCur);
    if (it_v != loopVisualIndexCheckedMap.end())
    {
        int loop_pre_candidate = it_v->second;
        if (abs(loop_pre_candidate - loopKeyPre) < filterNodeNum)
            return false;
    }
    // the loop pairs exits in the rgb detect container
    auto it_r = loopRGBVisualIndexCheckedMap.find(loopKeyCur);
    if (it_r != loopRGBVisualIndexCheckedMap.end())
    {
        int loop_pre_candidate = it_r->second;
        if (abs(loop_pre_candidate - loopKeyPre) < filterNodeNum)
            return false;
    }

    if (abs(lastLoopIndex - loopKeyCur) < filterNodeNum && lastLoopIndex != -1)
        return false;

    // total accumuted distance bnetween 2 frame
    /*    if (oldMeasurements.size() >= loopKeyCur) {
            double distance = 0.0;
            for (int j = loopKeyPre; j < loopKeyCur; ++j) {
                distance += oldMeasurements.at(j).distance;
            }
            // LOG(INFO) << "TOTAL DIS:" << distance;
            if (distance < filterDis) {
                std::cout << "CLOSE FRAME MUST FILTER OUT : " << distance << std::endl;
                return false;
            }
        }*/

    // stair case for hilti
    double z_offset = abs(oldMeasurements.at(loopKeyCur).updated_pose.z -
                          oldMeasurements.at(loopKeyPre).updated_pose.z);
    if (z_offset > LOOP_Z_OFFSET)
        return false;

    lastLoopIndex = loopKeyCur;

    return true;
}

bool MSMapping::DetectLoopClosureDistance(int *loopKeyCur,
                                          int *loopKeyPre)
{
    // 当前关键帧
    // int loopKeyCur = keyframePoses.size() - 1;
    // int loopKeyPre = -1;
    // 当前帧已经添加过闭环对应关系，不再继续添加
    //    auto it = loopIndexCheckedMap.find(*loopKeyCur);
    //    if (it != loopIndexCheckedMap.end())
    //        return false;
    //
    //    if (abs(lastLoopIndex - *loopKeyCur) < 5 && lastLoopIndex != -1)
    //        return false;

    //    std::unique_lock<std::mutex> kf_guard(mKF);
    keyframePoses2D.clear();
    //    keyframePoses3D.clear();
    for (int i = 0; i < oldMeasurements.size(); ++i)
    {
        Pose6D pose6D = oldMeasurements.at(i).updated_pose;
        //        keyframePoses3D.push_back(pose6D);
        pose6D.z = 0;
        keyframePoses2D.push_back(pose6D);
    }
    //    kf_guard.unlock();

    // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloudKeyPoses3D =
    //    vector2pc(keyframePoses3D);
    pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloudKeyPoses2D = vector2pc(keyframePoses2D);

    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    //    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
    //    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(),
    //                                        historyKeyframeSearchRadius,
    //                                        pointSearchIndLoop,
    //                                        pointSearchSqDisLoop,
    //                                        0);
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses2D);
    kdtreeHistoryKeyPoses->radiusSearch(
        copy_cloudKeyPoses2D->at(*loopKeyCur), historyKeyframeSearchRadius,
        pointSearchIndLoop, pointSearchSqDisLoop, 0);

    // 在候选关键帧集合中，找到与当前帧时间相隔较远的帧，设为候选匹配帧
    for (int i = 0; i < pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop.at(i);
        if (abs(oldMeasurements.at(id).odom_time -
                oldMeasurements.at(*loopKeyCur).odom_time) >
            historyKeyframeSearchTimeDiff)
        {
            *loopKeyPre = id;
            break;
        }
    }

    //    LOG(INFO) << "TIMES DISTANCE keyframePoses2D SIZE: " <<
    //              keyframeTimes.size() << ", " << keyframeDistances.size() << ",
    //              " << copy_cloudKeyPoses2D->size() << ", "
    //              << keyframePoses2D.size();
    if (*loopKeyPre == -1 || *loopKeyCur == *loopKeyPre)
        return false;

    if (!FilterLoopPairs(*loopKeyCur, *loopKeyPre))
        return false;
    return true;
}

void MSMapping::VisualizeLoopConstrains(std::map<int, int> loopMap,
                                        ros::Publisher &publisher,
                                        int type)
{
    if (loopMap.empty())
        return;

    visualization_msgs::MarkerArray markerArray;
    // 闭环顶点
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odom_link; // camera_init
    markerNode.header.stamp = ros::Time().fromSec(oldMeasurements.back().odom_time);
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.2;
    markerNode.scale.y = 0.2;
    markerNode.scale.z = 0.2;

    // 闭环边
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = odom_link;
    markerEdge.header.stamp = ros::Time().fromSec(oldMeasurements.back().odom_time);
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.2;
    markerEdge.scale.y = 0.2;
    markerEdge.scale.z = 0.2;

    switch (type)
    {
    case -3:
        markerNode.ns = markerEdge.ns = "gravity";
        markerNode.color.r = markerEdge.color.r = 0.1;
        markerNode.color.g = markerEdge.color.g = 0.7;
        markerNode.color.b = markerEdge.color.b = 0.2;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    case -2:
        markerNode.ns = markerEdge.ns = "map_constraint_nodes";
        markerNode.color.r = markerEdge.color.r = 0.6;
        markerNode.color.g = markerEdge.color.g = 0.1;
        markerNode.color.b = markerEdge.color.b = 0.2;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    case -1:
        // gps
        markerNode.ns = markerEdge.ns = "gps_nodes";
        markerNode.color.r = markerEdge.color.r = 0.0;
        markerNode.color.g = markerEdge.color.g = 1.0;
        markerNode.color.b = markerEdge.color.b = 0.1;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    case 0:
        // icp
        markerNode.ns = markerEdge.ns = "lidar_nodes";
        markerNode.color.r = markerEdge.color.r = 0.9;
        markerNode.color.g = markerEdge.color.g = 0.9;
        markerNode.color.b = markerEdge.color.b = 0;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    case 1:
        // icp
        markerNode.ns = markerEdge.ns = "sc_nodes";
        markerNode.color.r = markerEdge.color.r = 1;
        markerNode.color.g = markerEdge.color.g = 0.1;
        markerNode.color.b = markerEdge.color.b = 0.6;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    case 2:
        // intensity
        markerNode.ns = markerEdge.ns = "intensity_nodes";
        markerNode.color.r = markerEdge.color.r = 1;
        markerNode.color.g = markerEdge.color.g = 0;
        markerNode.color.b = markerEdge.color.b = 1;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    case 3:
        // rgb
        markerNode.ns = markerEdge.ns = "rgb_loop_nodes";
        markerNode.color.r = markerEdge.color.r = 0.6;
        markerNode.color.g = markerEdge.color.g = 0.95;
        markerNode.color.b = markerEdge.color.b = 0.6;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    default:
        std::cout << "error visulizer type!!!" << std::endl;
    }

    for (auto it = loopMap.begin(); it != loopMap.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;

        geometry_msgs::Point p;
        p.x = oldMeasurements.at(key_cur).updated_pose.x;
        p.y = oldMeasurements.at(key_cur).updated_pose.y;
        p.z = oldMeasurements.at(key_cur).updated_pose.z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);

        if (type == -1)
        {
            p.x = keyframeGPS.at(key_pre).x;
            p.y = keyframeGPS.at(key_pre).y;
            p.z = keyframeGPS.at(key_pre).z;
        }
        if (type == -2)
        {
            p.x = oldMeasurements.at(key_pre).global_pose.x;
            p.y = oldMeasurements.at(key_pre).global_pose.y;
            p.z = oldMeasurements.at(key_pre).global_pose.z;
        }
        else
        {
            p.x = oldMeasurements[key_pre].updated_pose.x;
            p.y = oldMeasurements[key_pre].updated_pose.y;
            p.z = oldMeasurements[key_pre].updated_pose.z;
        }
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);

    publisher.publish(markerArray);
}

void MSMapping::VisualizeLoopConstrains(std::map<int, std::vector<int>> loopMap,
                                        ros::Publisher &publisher,
                                        int type)
{
    if (loopMap.empty())
        return;

    visualization_msgs::MarkerArray markerArray;
    // 闭环顶点
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odom_link; // camera_init
    markerNode.header.stamp = ros::Time().fromSec(oldMeasurements.back().odom_time);
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.2;
    markerNode.scale.y = 0.2;
    markerNode.scale.z = 0.2;

    // 闭环边
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = odom_link;
    markerEdge.header.stamp = ros::Time().fromSec(oldMeasurements.back().odom_time);
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.2;
    markerEdge.scale.y = 0.2;
    markerEdge.scale.z = 0.2;

    switch (type)
    {
    case -3:
        markerNode.ns = markerEdge.ns = "gravity";
        markerNode.color.r = markerEdge.color.r = 0.1;
        markerNode.color.g = markerEdge.color.g = 0.7;
        markerNode.color.b = markerEdge.color.b = 0.2;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    case -2:
        markerNode.ns = markerEdge.ns = "map_constraint_nodes";
        markerNode.color.r = markerEdge.color.r = 0.6;
        markerNode.color.g = markerEdge.color.g = 0.1;
        markerNode.color.b = markerEdge.color.b = 0.2;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    case -1:
        // gps
        markerNode.ns = markerEdge.ns = "gps_nodes";
        markerNode.color.r = markerEdge.color.r = 0.0;
        markerNode.color.g = markerEdge.color.g = 1.0;
        markerNode.color.b = markerEdge.color.b = 0.1;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    case 0:
        // icp
        markerNode.ns = markerEdge.ns = "lidar_nodes";
        markerNode.color.r = markerEdge.color.r = 0.9;
        markerNode.color.g = markerEdge.color.g = 0.9;
        markerNode.color.b = markerEdge.color.b = 0;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    case 1:
        // icp
        markerNode.ns = markerEdge.ns = "sc_nodes";
        markerNode.color.r = markerEdge.color.r = 1;
        markerNode.color.g = markerEdge.color.g = 0.1;
        markerNode.color.b = markerEdge.color.b = 0.6;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    case 2:
        // intensity
        markerNode.ns = markerEdge.ns = "intensity_nodes";
        markerNode.color.r = markerEdge.color.r = 1;
        markerNode.color.g = markerEdge.color.g = 0;
        markerNode.color.b = markerEdge.color.b = 1;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    case 3:
        // rgb
        markerNode.ns = markerEdge.ns = "rgb_loop_nodes";
        markerNode.color.r = markerEdge.color.r = 0.6;
        markerNode.color.g = markerEdge.color.g = 0.95;
        markerNode.color.b = markerEdge.color.b = 0.6;
        markerNode.color.a = markerEdge.color.a = 1;
        break;
    default:
        std::cout << "error visulizer type!!!" << std::endl;
    }

    for (const auto &pair : loopMap)
    {
        int key_cur = pair.first;
        const std::vector<int> &keys_pre = pair.second;
        for (int key_pre : keys_pre)
        {
            geometry_msgs::Point p;
            p.x = oldMeasurements.at(key_cur).updated_pose.x;
            p.y = oldMeasurements.at(key_cur).updated_pose.y;
            p.z = oldMeasurements.at(key_cur).updated_pose.z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            if (type == -1)
            {
                p.x = keyframeGPS.at(key_pre).x;
                p.y = keyframeGPS.at(key_pre).y;
                p.z = keyframeGPS.at(key_pre).z;
            }
            else if (type == -2)
            {
                p.x = oldMeasurements.at(key_pre).global_pose.x;
                p.y = oldMeasurements.at(key_pre).global_pose.y;
                p.z = oldMeasurements.at(key_pre).global_pose.z;
            }
            else
            {
                p.x = oldMeasurements[key_pre].updated_pose.x;
                p.y = oldMeasurements[key_pre].updated_pose.y;
                p.z = oldMeasurements[key_pre].updated_pose.z;
            }
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);

    publisher.publish(markerArray);
}

bool MSMapping::GetGlobalICP(Measurement &measurement_temp)
{
    // use this pose, do ndt
    Pose3 poseGlobalPre =
        Pose6dTogtsamPose3(keyMeasures.at(prev_node_idx).global_pose);

    NavState predictState(prevPose, prevVel);
    NavState predictGlobalState(poseGlobalPre, prevVel);
    NavState prevGlobalState(poseGlobalPre, prevVel);

    if (0)
    {
        std::cout << BOLDCYAN << "prevPose: " << prevPose.translation().transpose()
                  << std::endl;
        std::cout << BOLDCYAN
                  << "prevGlobalPose: " << poseGlobalPre.translation().transpose()
                  << std::endl;
        //    std::cout << BOLDCYAN << "predictGlobalState: "
        //              << predictGlobalState.pose().translation().transpose()
        //              << std::endl;
        std::cout << BOLDCYAN << "predictState: "
                  << propState.pose().translation().transpose() << std::endl;
    }
    Eigen::Matrix4d guess_matrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d final_trans = Eigen::Matrix4d::Identity();

    // use pgo pose
    //     guess_matrix = Pose6D2Matrix(measurement_temp.updated_pose).matrix();
    // use previou global pose
    // guess_matrix = predictGlobalState.pose().matrix();
    // use predict pose by pgo
    //    guess_matrix = predictState.pose().matrix();
    //    guess_matrix = (prevPose).matrix();
    guess_matrix = predict_pose.matrix();
    //    guess_matrix = predictGlobalState.pose().matrix();
    //

    // Extract the local map
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_map(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*globalmap_filter_ptr, *temp_map);
    // VoxelGrid downsampling
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(temp_map);
    sor.setLeafSize(0.5, 0.5, 0.5);
    sor.filter(*temp_map);
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map = cloud_process_.extractLocalMap(temp_map, predict_pose.matrix(), 80);
    std::cout << "Extracted " << local_map->size() << " points from global map. " << globalmap_filter_ptr->size()
              << std::endl;

    //    pcl::PointCloud<PointT>::Ptr crop_cloud_map(new pcl::PointCloud<PointT>());
    //    cloud_process_.CropGlobalCloud(globalmap_filter_extracted_ptr, crop_cloud_map, guess_matrix,
    //                                   Eigen::Vector3d(100, 100, 100));
    if (local_map->empty())
        return false;
    publishCloud(pubLaserCloudCrop, local_map, ros::Time::now(), odom_link);

    bool useOpen3d = true;
    double score = 9999999;
    if (useOpen3d)
    {
        //*******************open 3d pcl ****************
        std::shared_ptr<geometry::PointCloud> source_o3d = cloud_process_.GetO3dPointCloudFromPCL(*measurement_temp.lidar);
        std::shared_ptr<geometry::PointCloud> target_o3d = cloud_process_.GetO3dPointCloudFromPCL(*local_map);

        TicToc tic;
        pipelines::registration::RegistrationResult icp;
        auto criteria = pipelines::registration::ICPConvergenceCriteria(20);
        switch (method)
        {
        case 0: // point-to-point icp
            icp = pipelines::registration::RegistrationICP(
                *source_o3d, *target_o3d, 1.0, guess_matrix.cast<double>(),
                pipelines::registration::TransformationEstimationPointToPoint(),
                criteria);
            break;
        case 1: // Point-to-plane
            source_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(2.0, 5));
            target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(2.0, 5));
            icp = pipelines::registration::RegistrationICP(
                *source_o3d, *target_o3d, 1.0, guess_matrix.cast<double>(),
                pipelines::registration::TransformationEstimationPointToPlane(),
                criteria);
            break;
        case 2:
            source_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(2.0, 5));
            target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(2.0, 5));
            icp = pipelines::registration::RegistrationGeneralizedICP(
                *source_o3d, *target_o3d, 1.0, guess_matrix.cast<double>(),
                pipelines::registration::
                    TransformationEstimationForGeneralizedICP(),
                criteria);
            break;
        default:
            std::cout << " evaluation error type!!!!! " << std::endl;
            break;
        }

        final_trans = icp.transformation_;
        double score = icp.inlier_rmse_;
        double overlap = icp.fitness_;

        // measurement_temp.global_pose = Matrix2Pose6D(trans);
        measurement_temp.global_pose = Matrix2Pose6D(final_trans);
        measurement_temp.global_score = score;

        // 使用 EigenMatrixToTensor 转换 Eigen 矩阵到 Open3D Tensor
        open3d::core::Tensor transformation_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(
            icp.transformation_);
        double max_correspondence_distance = 1.0;
        open3d::t::geometry::PointCloud source_o3d_new = open3d::t::geometry::PointCloud::FromLegacy(*source_o3d);
        open3d::t::geometry::PointCloud target_o3d_new = open3d::t::geometry::PointCloud::FromLegacy(*target_o3d);
        open3d::core::Tensor information_matrix = open3d::t::pipelines::registration::GetInformationMatrix(
            source_o3d_new,
            target_o3d_new,
            max_correspondence_distance,
            transformation_tensor);
        Eigen::MatrixXd information_matrix_eigen = open3d::core::eigen_converter::TensorToEigenMatrixXd(
            information_matrix);
        if (information_matrix_eigen.rows() == 6 && information_matrix_eigen.cols() == 6)
        {
            icp_cov = information_matrix_eigen.inverse().cast<float>() * 1e-3;
        }
        else
        {
            // 处理错误情况：信息矩阵不是 6x6 矩阵
            std::cerr << "Information matrix is not 6x6. Cannot compute covariance matrix." << std::endl;
        }
        std::cout << "O3D MAP COV: \n"
                  << icp_cov.diagonal().transpose() * 1e6 << std::endl;

        // publish global odomertry
        if (1)
        {
            std::cout << BOLDMAGENTA << "Global ICP ALIGNED POINTS: "
                      << measurement_temp.lidar->size()
                      << " " << local_map->size() << " "
                      << score << " " << overlap << std::endl;
        }
        if (score == 0.0 || score > 0.3 || overlap < 0.8)
        {
            measurement_temp.global_pose.valid = false;
            return false;
        }
    }
    else
    {
        // **********************pcl icp **************************
        // ICP Settings
        static pcl::IterativeClosestPoint<PointT, PointT> icp;
        // pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
        icp.setMaxCorrespondenceDistance(1.0);
        icp.setMaximumIterations(30);
        icp.setRANSACIterations(0);

        // Align pointclouds
        icp.setInputSource(measurement_temp.lidar);
        icp.setInputTarget(local_map);
        //    icp.setInputTarget(globalmap_ptr);
        pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
        icp.align(*unused_result, guess_matrix.cast<float>());

        final_trans = icp.getFinalTransformation().cast<double>();
        score = icp.getFitnessScore();
        bool converged = icp.hasConverged();

        // measurement_temp.global_pose = Matrix2Pose6D(trans);
        measurement_temp.global_pose = Matrix2Pose6D(final_trans);
        measurement_temp.global_score = score;

        if (1)
        {
            std::cout << BOLDMAGENTA << "Global ICP ALIGNED POINTS: "
                      << measurement_temp.lidar->size() << " "
                      << local_map->size() << " " << score << " " << converged
                      << std::endl;
        }

        if (icp.hasConverged() == false || icp.getFitnessScore() > 0.3)
        {
            return false;
        }
    }
    return true;
}

void MSMapping::PublishPose(const ros::Time &time,
                            const ros::Publisher &topic_pub,
                            const std::string &base_frame_id,
                            const Eigen::Matrix4d &transform_matrix)
{
    geometry_msgs::PoseWithCovarianceStamped pose_;
    pose_.header.stamp = time;
    pose_.header.frame_id = base_frame_id;

    // set the position
    pose_.pose.pose.position.x = transform_matrix(0, 3);
    pose_.pose.pose.position.y = transform_matrix(1, 3);
    pose_.pose.pose.position.z = transform_matrix(2, 3);

    Eigen::Quaterniond q;
    q = transform_matrix.block<3, 3>(0, 0);
    pose_.pose.pose.orientation.x = q.x();
    pose_.pose.pose.orientation.y = q.y();
    pose_.pose.pose.orientation.z = q.z();
    pose_.pose.pose.orientation.w = q.w();

    topic_pub.publish(pose_);
}

Vector28 MSMapping::GetStateFromLIO(const int node_id)
{
    auto &latest_measurement = keyMeasures.at(node_id);
    Vector28 state;
    state.setZero();
    state << latest_measurement.odom.pose.pose.position.x,
        latest_measurement.odom.pose.pose.position.y,
        latest_measurement.odom.pose.pose.position.z,
        keyMeasures.at(node_id).key_pose.roll,
        keyMeasures.at(node_id).key_pose.pitch,
        keyMeasures.at(node_id).key_pose.yaw,

        // vel
        latest_measurement.odom.twist.twist.linear.x,
        latest_measurement.odom.twist.twist.linear.y,
        latest_measurement.odom.twist.twist.linear.z,

        // bias a and g
        latest_measurement.odom.twist.covariance[4],
        latest_measurement.odom.twist.covariance[5],
        latest_measurement.odom.twist.covariance[6],
        latest_measurement.odom.twist.covariance[1],
        latest_measurement.odom.twist.covariance[2],
        latest_measurement.odom.twist.covariance[3],

        // gravity
        latest_measurement.odom.twist.covariance[8],
        latest_measurement.odom.twist.covariance[9],
        latest_measurement.odom.twist.covariance[10],

        // pose cov
        latest_measurement.odom.pose.covariance[0],
        latest_measurement.odom.pose.covariance[7],
        latest_measurement.odom.pose.covariance[14],
        latest_measurement.odom.pose.covariance[21],
        latest_measurement.odom.pose.covariance[28],
        latest_measurement.odom.pose.covariance[35],

        // vel cov
        latest_measurement.odom.pose.covariance[1],
        latest_measurement.odom.pose.covariance[2],
        latest_measurement.odom.pose.covariance[3],

        // ba bg cov
        latest_measurement.odom.pose.covariance[8],
        latest_measurement.odom.pose.covariance[9],
        latest_measurement.odom.pose.covariance[10],
        latest_measurement.odom.pose.covariance[4],
        latest_measurement.odom.pose.covariance[5],
        latest_measurement.odom.pose.covariance[6],

        // gravity
        latest_measurement.odom.pose.covariance[11],
        latest_measurement.odom.pose.covariance[12],
        1e-6;

    return state;
}

State MSMapping::GetStateFromLIO2(const int node_id)
{
    const auto &latest_measurement = keyMeasures.at(node_id).odom; // Assuming this is a nav_msgs::Odometry
    const auto &key_measure = keyMeasures.at(node_id);

    State state;
    // Get the orientation quaternion from the message
    const auto &orientation = latest_measurement.pose.pose.orientation;
    gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);

    // Get the position from the message
    const auto &position = latest_measurement.pose.pose.position;
    gtsam::Point3 translation(position.x, position.y, position.z);

    // Assign pose
    state.pose = gtsam::Pose3(rotation, translation);
    // Get the linear velocity from the message
    const auto &linear_velocity = latest_measurement.twist.twist.linear;
    state.vel = gtsam::Vector3(linear_velocity.x, linear_velocity.y, linear_velocity.z);

    // Reorder the covariance matrix to match the State format
    const auto &odom_covariance = latest_measurement.pose.covariance;
    gtsam::Matrix6 &state_cov = state.cov;

    // Switch the ordering of translation and rotation
    //    for (int i = 0; i < 6; ++i) {
    //        int row = i < 3 ? i + 3 : i - 3; // switch rows for translation and rotation
    //        for (int j = 0; j < 6; ++j) {
    //            int col = j < 3 ? j + 3 : j - 3; // switch columns for translation and rotation
    //            state_cov(i, j) = odom_covariance[row * 6 + col];
    //        }
    //    }

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            state_cov(i, j) = odom_covariance[i * 6 + j];
        }
    }

    return state;
}
/*******************************************************
 * Author: Xiangcheng Hu (xhubd@connect.ust.hk.com)
 * Date: ${Date}
 * Description:
 *******************************************************/
#include "cloud_process.h"

pcl::PointCloud<pcl::PointXYZI>::Ptr CloudProcess::RemoveRangeCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Vector3i axis,
    Eigen::Vector3d threshold, std::string op)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond(
        new pcl::ConditionAnd<pcl::PointXYZI>());
    pcl::ComparisonOps::CompareOp oper;
    if (op == ">")
        oper = pcl::ComparisonOps::LT;
    else
        oper = pcl::ComparisonOps::GT;

    if (axis == Eigen::Vector3i(1, 1, 1))
    {
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZI>("x", oper, threshold[0])));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZI>("y", oper, threshold[1])));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZI>("z", oper, threshold[2])));
    }
    else if (axis == Eigen::Vector3i(1, 1, 0))
    {
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZI>("x", oper, threshold[0])));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZI>("y", oper, threshold[1])));
    }
    else
    {
        std::cout << "conditional cloud filter can not support this type!"
                  << std::endl;
    }

    pcl::ConditionalRemoval<pcl::PointXYZI> condrem(true);
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(false);
    condrem.filter(*filtered);
    return filtered;
}

void CloudProcess::CropGlobalCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr &globalmap_ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &transform_cloud_ptr,
    Eigen::Matrix4d &trans, Eigen::Vector3d size)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_map(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr crop_cloud_map_filtered(
        new pcl::PointCloud<pcl::PointXYZI>);

    pcl::copyPointCloud(*globalmap_ptr, *temp_map);
    // transform point cloud to lidar frame
    // Pose6D map2base_curr = Matrix2Pose6D(Pose6D2Matrix(pose).inverse());
    //  pcl::transformPointCloud(*temp_map, *transform_cloud_ptr,
    //                           pose.inverse().cast<float>());
    *transform_cloud_ptr = *TransformPointCloud(temp_map, trans.inverse());

    // to lidar frame
    //*transform_cloud_ptr = *Local2global(temp_map,
    // Matrix2Pose6D(Pose6D2Matrix(pose).inverse()));
    crop_cloud_map_filtered = RemoveRangeCloud(
        transform_cloud_ptr, Eigen::Vector3i(1, 1, 1), size, ">");
    //  crop_cloud_map_filtered = range_remove(
    //      crop_cloud_map_filtered, Eigen::Vector3i(20, 20, 20), size, "<");

    // map link
    // pcl::PointCloud<PointT>::Ptr trans_cloud_map(new
    // pcl::PointCloud<PointT>());
    transform_cloud_ptr->clear();
    //  pcl::transformPointCloud(*crop_cloud_map_filtered, *transform_cloud_ptr,
    //                           pose.cast<float>());

    *transform_cloud_ptr = *TransformPointCloud(crop_cloud_map_filtered, trans);

    //  publishCloud(pubLaserCloudCrop, transform_cloud_ptr, ros::Time::now(),
    //               odom_link);

    // *transform_cloud_ptr = *trans_cloud_map;
    //    std::cout << BOLDWHITE << "crop global map: " <<
    //    crop_cloud_map_filtered->size() << ", " << transform_cloud_ptr->size()
    //              << std::endl;
}

void CloudProcess::AddNormal(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(
        new pcl::search::KdTree<pcl::PointXYZI>);
    searchTree->setInputCloud(cloud);

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud);
    normalEstimator.setSearchMethod(searchTree);
    normalEstimator.setKSearch(15);
    normalEstimator.compute(*normals);

    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
}

std::shared_ptr<open3d::geometry::PointCloud>
CloudProcess::GetO3dPointCloudFromPCL(pcl::PointCloud<pcl::PointXYZI> &pc)
{
    std::shared_ptr<open3d::geometry::PointCloud> point_cloud_p3d(
        new open3d::geometry::PointCloud());
    point_cloud_p3d->points_.resize(pc.size());
    for (int i = 0; i < pc.size(); ++i)
    {
        point_cloud_p3d->points_[i][0] = pc.points.at(i).x;
        point_cloud_p3d->points_[i][1] = pc.points.at(i).y;
        point_cloud_p3d->points_[i][2] = pc.points.at(i).z;
        //    point_cloud_p3d->colors_.at(i)
    }
    return point_cloud_p3d;
}

void CloudProcess::FindNearKeyframes(
    std::vector<Measurement> &keyMeasures,
    pcl::PointCloud<PointT>::Ptr &nearKeyframes, const int &key,
    const int &searchNum)
{
    // 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合
    nearKeyframes->clear();
    int cloudSize = keyMeasures.size();
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize)
            continue;
        // *nearKeyframes += *transformPointCloud(keyframeLaserClouds[keyNear],
        // &copy_cloudKeyPoses6D->points[keyNear]);
        *nearKeyframes += *TransformPointCloud(keyMeasures[keyNear].lidar,
                                               keyMeasures[keyNear].updated_pose);
    }

    if (nearKeyframes->empty())
        return;

    //    scan_filter_size = 0.1;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterICP;
    downSizeFilterICP.setLeafSize(scan_filter_size * 2, scan_filter_size * 2,
                                  scan_filter_size * 2);

    if (searchNum == 0)
    {
        *nearKeyframes = *nearKeyframes;
    }
    else
    {
        pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>());
        if (useRawCloud)
            downSizeFilterICP.setLeafSize(scan_filter_size, scan_filter_size,
                                          scan_filter_size);

        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*nearKeyframes);
        //        *nearKeyframes = *cloud_temp;
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
CloudProcess::CreateGlobalMap(const std::vector<Measurement> &measurements,
                              const std::vector<int> &representative_indices,
                              const std::string &cloud_directory,
                              int frames_per_trajectory)
{
    using namespace pcl;
    std::vector<PointCloud<PointXYZI>::Ptr> localmaps;
    PointCloud<PointXYZI>::Ptr global_map(new PointCloud<PointXYZI>);
    for (int index : representative_indices)
    {
        int start_frame = std::max(0, index - frames_per_trajectory);
        int end_frame = std::min(static_cast<int>(measurements.size()) - 1, index + frames_per_trajectory);
        global_map->clear();
        for (int i = start_frame; i <= end_frame; ++i)
        {
            PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>);
            if (measurements[i].lidar->empty())
            {
                std::string cloud_file = cloud_directory + std::to_string(i) + ".pcd";
                if (io::loadPCDFile<PointXYZI>(cloud_file, *cloud) == -1)
                {
                    continue;
                }
            }
            else
            {
                cloud = measurements[i].lidar;
            }
            Eigen::Matrix4f transformation = Pose6D2Matrix(measurements[i].updated_pose).cast<float>();
            transformPointCloud(*cloud, *cloud, transformation);
            *global_map += *cloud;
        }
        // 下采样全局地图
        PointCloud<PointXYZI>::Ptr downsampled(new PointCloud<PointXYZI>);
        VoxelGrid<PointXYZI> sor;
        sor.setInputCloud(global_map);
        sor.setLeafSize(0.1f, 0.1f, 0.1f);
        sor.filter(*downsampled);
        localmaps.push_back(downsampled);
        std::cout << "GET LOCALMAP " << index << ", " << downsampled->size() << " " << global_map->size();
    }
    return localmaps;
}

bool CloudProcess::PerformICP(pcl::PointCloud<PointT>::Ptr &cureKeyframeCloud,
                              pcl::PointCloud<PointT>::Ptr &targetKeyframeCloud,
                              Eigen::Matrix4d &final_trans,
                              double &score,
                              double &overlap)
{
    using namespace open3d;
    if (cureKeyframeCloud->size() < 100 || targetKeyframeCloud->size() < 100)
        return false;

    //*******************open 3d pcl ****************
    std::shared_ptr<geometry::PointCloud> source_o3d =
        GetO3dPointCloudFromPCL(*cureKeyframeCloud);
    std::shared_ptr<geometry::PointCloud> target_o3d =
        GetO3dPointCloudFromPCL(*targetKeyframeCloud);
    //    source_o3d = source_o3d->VoxelDownSample(0.2);
    //    target_o3d = target_o3d->VoxelDownSample(0.5);

    overlap = 0.0, score = 0.0;
    double max_correspondence_distance = 2.0; // 根据需求设置
    pipelines::registration::RegistrationResult icp;
    auto criteria = pipelines::registration::ICPConvergenceCriteria(20);
    switch (method)
    {
    case 0: // point-to-point icp
    {
        icp = pipelines::registration::RegistrationICP(
            *source_o3d, *target_o3d, max_correspondence_distance, Eigen::Matrix4d::Identity(),
            pipelines::registration::TransformationEstimationPointToPoint(),
            criteria);
        final_trans = icp.transformation_;
        score = icp.inlier_rmse_;
        overlap = icp.fitness_;
        break;
    }
    case 1: // Point-to-plane
    {
        target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(max_correspondence_distance, 20));
        icp = pipelines::registration::RegistrationICP(
            *source_o3d, *target_o3d, max_correspondence_distance, Eigen::Matrix4d::Identity(),
            pipelines::registration::TransformationEstimationPointToPlane(),
            criteria);
        final_trans = icp.transformation_;
        score = icp.inlier_rmse_;
        overlap = icp.fitness_;
        break;
    }
    case 2:
    {
        target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(max_correspondence_distance, 20));
        icp = pipelines::registration::RegistrationGeneralizedICP(
            *source_o3d, *target_o3d, max_correspondence_distance, Eigen::Matrix4d::Identity(),
            pipelines::registration::TransformationEstimationForGeneralizedICP(),
            criteria);
        final_trans = icp.transformation_;
        score = icp.inlier_rmse_;
        overlap = icp.fitness_;
        break;
    }
    case 3:
    {
        // std::cout << "USE MULTI SCALE ICP!" << std::endl;
        // 多尺度ICP
        // 将 Open3D 的标准点云转换为张量点云
        target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(max_correspondence_distance, 20));
        auto source_tensor = open3d::t::geometry::PointCloud::FromLegacy(*source_o3d).VoxelDownSample(0.1);
        auto target_tensor = open3d::t::geometry::PointCloud::FromLegacy(*target_o3d).VoxelDownSample(0.1);
        open3d::core::Tensor init_matrix = open3d::core::eigen_converter::EigenMatrixToTensor(
            Eigen::Matrix4d::Identity());

        // 设置多尺度参数
        std::vector<double> voxel_sizes = {0.5, 0.3, 0.1};
        std::vector<double> max_correspondence_distances = {5.0, 0.5, 0.25};
        std::vector<t::pipelines::registration::ICPConvergenceCriteria> criteria_list = {
            t::pipelines::registration::ICPConvergenceCriteria(0.01, 0.01, 15),
            t::pipelines::registration::ICPConvergenceCriteria(0.001, 0.001, 10),
            t::pipelines::registration::ICPConvergenceCriteria(0.000001, 0.000001, 5)};
        // 执行多尺度ICP
        auto multi_icp = t::pipelines::registration::MultiScaleICP(
            source_tensor, target_tensor, voxel_sizes, criteria_list,
            max_correspondence_distances, init_matrix,
            t::pipelines::registration::TransformationEstimationPointToPlane());

        final_trans = open3d::core::eigen_converter::TensorToEigenMatrixXd(multi_icp.transformation_);
        score = multi_icp.inlier_rmse_;
        overlap = multi_icp.fitness_;
        break;
    }
    default:
        std::cout << " evaluation error type!!!!! " << std::endl;
        return false;
    }
    if (score > loopFitnessScoreThreshold || overlap < 0.7 || score == 0.0)
    {
        std::cout << BOLDCYAN << "[LOOP] LOOP ICP FAILED Score and Overlap: " << source_o3d->points_.size() << " "
                  << target_o3d->points_.size() << " "
                  << score << " " << overlap << std::endl;
        return false;
    }
    return true;
}

bool CloudProcess::DoICPVirtualRelative3(std::vector<Measurement> &keyMeasures,
                                         pcl::PointCloud<PointT>::Ptr &targetKeyframeCloud,
                                         const std::string &mapDir,
                                         int loopKeyPre, int loopKeyCur,
                                         double &score, int type,
                                         Eigen::Matrix4d &final_trans)
{
    using namespace open3d;
    // int historyKeyframeSearchNum = 25; // enough. ex. [-25, 25] covers submap
    // length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointT>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointT>());
    targetKeyframeCloud.reset(new pcl::PointCloud<PointT>());
    //    pcl::PointCloud<PointT>::Ptr targetKeyframeCloud(
    //            new pcl::PointCloud<PointT>());
    // historyKeyframeSearchNum, _loop_kf_idx); 提取当前关键帧特征点集合，降采样
    FindNearKeyframes(keyMeasures, cureKeyframeCloud, loopKeyCur, 0);

    // 提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
    // FindNearKeyframes(keyMeasures, targetKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
    targetKeyframeCloud = CreateLocalMapFromMeasurements(keyMeasures, loopKeyPre, loopKeyCur, mapDir, historyKeyframeSearchNum);

    // std::cout << "Get history target cloud: " << targetKeyframeCloud->size() << std::endl;
    if (cureKeyframeCloud->size() < 100 || targetKeyframeCloud->size() < 100)
        return false;

    double overlap = 0.0;
    Eigen::Matrix4d icp_trans = Eigen::Matrix4d::Identity();
    pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
    bool isLoopClosed = PerformICP(cureKeyframeCloud, targetKeyframeCloud, icp_trans, score, overlap);
    if (!isLoopClosed)
        return false;
    *unused_result = *TransformPointCloud(cureKeyframeCloud, icp_trans);
    final_trans = icp_trans * final_trans;
    return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
CloudProcess::CreateLocalMapFromMeasurements(const std::vector<Measurement> &measurements,
                                             int history_frame_index,
                                             int curr_frame_index,
                                             const std::string &cloud_directory,
                                             int num_closest_poses)
{
    using namespace pcl;
    // 检查索引有效性
    if (history_frame_index < 0 || history_frame_index >= measurements.size())
    {
        throw std::runtime_error("Invalid history_frame_index.");
    }
    // 获取历史帧的位姿
    Pose6D history_pose = measurements[history_frame_index].updated_pose;
    // 定义优先队列
    using PoseDistance = std::pair<double, int>;
    auto compare = [](const PoseDistance &a, const PoseDistance &b)
    { return a.first > b.first; };
    std::priority_queue<PoseDistance, std::vector<PoseDistance>, decltype(compare)> pq(compare);

    // 遍历所有位姿，计算距离并添加到优先队列中
    for (int i = 0; i < measurements.size(); ++i)
    {
        Pose6D pose = measurements[i].updated_pose;
        double time_difference = std::abs(measurements[i].odom_time - measurements[curr_frame_index].odom_time);
        // 只考虑时间上距离历史帧至少100秒的位姿
        if (time_difference >= 30.0)
        {
            Eigen::Vector2d pose_xy(pose.x, pose.y);
            Eigen::Vector2d history_pose_xy(history_pose.x, history_pose.y);
            double distance = (pose_xy - history_pose_xy).norm();
            pq.push(std::make_pair(distance, i));
        }
        //        Eigen::Vector2d pose_xy(pose.x, pose.y);
        //        Eigen::Vector2d history_pose_xy(history_pose.x, history_pose.y);
        //        double distance = (pose_xy - history_pose_xy).norm();
        //        pq.push(std::make_pair(distance, i));
    }

    // 收集最近的200个位姿
    std::vector<int> closest_indices;
    while (!pq.empty() && closest_indices.size() < num_closest_poses)
    {
        closest_indices.push_back(pq.top().second);
        pq.pop();
    }
    // 加载和拼接点云
    PointCloud<PointXYZI>::Ptr local_map(new PointCloud<PointXYZI>);
    PointCloud<PointXYZI>::Ptr transform_cloud(new PointCloud<PointXYZI>);
    for (int index : closest_indices)
    {
        if (measurements[index].lidar->empty())
        {
            // 如果没有点云，则从文件中加载
            std::string cloud_file = cloud_directory + std::to_string(index) + ".pcd";
            if (io::loadPCDFile<PointXYZI>(cloud_file, *transform_cloud) == -1)
            {
                std::cout << "read cloud failed: " << cloud_file << " " << transform_cloud->size() << std::endl;
                continue;
            }
            *measurements[index].lidar = *transform_cloud;
        }
        //        VoxelGrid<PointXYZI> sor;
        //        sor.setInputCloud(transform_cloud);
        //        sor.setLeafSize(0.1f, 0.1f, 0.1f);
        //        sor.filter(*transform_cloud);

        // 转换点云到全局坐标系
        Eigen::Matrix4f transformation = Pose6D2Matrix(
                                             measurements.at(index).updated_pose)
                                             .cast<float>();
        transformPointCloud(*measurements[index].lidar, *transform_cloud, transformation);
        *local_map += *transform_cloud;
    }

    // 下采样局部地图
    PointCloud<PointXYZI>::Ptr downsampled(new PointCloud<PointXYZI>);
    VoxelGrid<PointXYZI> sor;
    sor.setInputCloud(local_map);
    // sor.setLeafSize(0.2f, 0.2f, 0.2f);
    sor.setLeafSize(0.4f, 0.4f, 0.4f);
    sor.filter(*downsampled);
    // std::cout << "target cloud size: " << downsampled->size() << " " << local_map->size() << std::endl;
    return downsampled;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CloudProcess::CreateLocalMap(const Eigen::Matrix4d &initial_pose_matrix,
                                                                  const gtsam::Values &initial_values,
                                                                  const std::string &cloud_directory,
                                                                  int &closest_pose_index,
                                                                  int num_closest_poses)
{
    using namespace gtsam;
    using namespace pcl;

    // Convert initial pose matrix to GTSAM Pose3
    Pose3 initial_pose = Matrix4dToPose3(initial_pose_matrix);
    // std::cout << "initial pose2: " << initial_pose.translation().transpose() << std::endl;

    // Find the closest pose
    double min_distance = std::numeric_limits<double>::max();
    closest_pose_index = -1; // 初始化为-1

    // 定义一个优先队列来存储距离和对应的位姿
    using PoseDistance = std::pair<double, gtsam::Key>;
    auto compare = [](const PoseDistance &a, const PoseDistance &b)
    { return a.first > b.first; };
    std::priority_queue<PoseDistance, std::vector<PoseDistance>, decltype(compare)> pq(compare);

    // 遍历所有位姿，计算距离并添加到优先队列中
    for (const auto &key_value : initial_values)
    {
        gtsam::Symbol symbol(key_value.key);
        Pose3 pose = key_value.value.cast<Pose3>();
        Eigen::Vector2d pose_xy(pose.x(), pose.y());
        Eigen::Vector2d initial_pose_xy(initial_pose.x(), initial_pose.y());
        double distance = (pose_xy - initial_pose_xy).norm();
        pq.push(std::make_pair(distance, key_value.key));
        // 更新最近位姿的序号
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_pose_index = symbol.index();
        }
    }
    std::cout << "Find closest distance and index: " << min_distance << " " << closest_pose_index << std::endl;

    // 收集最近的200个位姿
    std::vector<gtsam::Key> closest_poses;
    while (!pq.empty() && closest_poses.size() < num_closest_poses)
    {
        closest_poses.push_back(pq.top().second);
        pq.pop();
    }

    // Load point clouds and transform to global frame
    pcl::PointCloud<PointXYZI>::Ptr local_map(new pcl::PointCloud<PointXYZI>);
    for (Key key : closest_poses)
    {
        gtsam::Symbol symbol(key);
        Pose3 pose = initial_values.at<Pose3>(key);
        // std::cout << "add pose cloud: " << symbol.index() << ". " << pose.translation().transpose() << std::endl;
        //  Pose3 relative_pose = reference_pose.between(pose); // Compute relative transformation
        std::string cloud_file = cloud_directory + std::to_string(symbol.index()) + ".pcd";
        pcl::PointCloud<PointXYZI>::Ptr cloud(new pcl::PointCloud<PointXYZI>);
        if (pcl::io::loadPCDFile<PointXYZI>(cloud_file, *cloud) == -1)
        {
            std::cout << "load failed : " << cloud_file << std::endl;
            continue;
        }
        Eigen::Matrix4f transformation = pose.matrix().cast<float>();
        transformPointCloud(*cloud, *cloud, transformation);
        *local_map += *cloud;
    }

    // Downsample the local map
    pcl::PointCloud<PointXYZI>::Ptr downsampled(new pcl::PointCloud<PointXYZI>);
    VoxelGrid<PointXYZI> sor;
    sor.setInputCloud(local_map);
    sor.setLeafSize(0.4f, 0.4f, 0.4f);
    sor.filter(*downsampled);
    std::cout << "localmap : " << downsampled->size() << " " << local_map->size() << std::endl;
    return downsampled;
}

std::vector<std::tuple<int, gtsam::Pose3, pcl::PointCloud<PointT>::Ptr>> CloudProcess::CreateLocalMapMulti(
    const Eigen::Matrix4d &initial_pose_matrix, const gtsam::Values &initial_values,
    const std::string &cloud_directory, int &closest_pose_index, int num_closest_poses, int num_nearby_frames)
{
    using namespace gtsam;
    using namespace pcl;

    // Convert initial pose matrix to GTSAM Pose3
    Pose3 initial_pose = Matrix4dToPose3(initial_pose_matrix);
    // Find the closest pose
    double min_distance = std::numeric_limits<double>::max();
    closest_pose_index = -1; // 初始化为-1
    // 定义一个优先队列来存储距离和对应的位姿
    using PoseDistance = std::pair<double, gtsam::Key>;
    auto compare = [](const PoseDistance &a, const PoseDistance &b)
    { return a.first > b.first; };
    std::priority_queue<PoseDistance, std::vector<PoseDistance>, decltype(compare)> pq(compare);
    // 遍历所有位姿,计算距离并添加到优先队列中
    for (const auto &key_value : initial_values)
    {
        gtsam::Symbol symbol(key_value.key);
        Pose3 pose = key_value.value.cast<Pose3>();
        Eigen::Vector2d pose_xy(pose.x(), pose.y());
        Eigen::Vector2d initial_pose_xy(initial_pose.x(), initial_pose.y());
        double distance = (pose_xy - initial_pose_xy).norm();
        pq.push(std::make_pair(distance, key_value.key));
        // 更新最近位姿的序号
        if (distance < 10.0)
        {
            min_distance = distance;
            closest_pose_index = symbol.index();
        }
    }
    std::cout << "Find closest distance and index: " << min_distance << " " << closest_pose_index << std::endl;
    // 收集最近的 5 个位姿
    std::vector<gtsam::Key> closest_poses;
    while (!pq.empty() && closest_poses.size() < num_closest_poses)
    {
        closest_poses.push_back(pq.top().second);
        pq.pop();
    }
    // 加载点云并转换到候选位姿对应的帧上
    std::vector<std::tuple<int, Pose3, pcl::PointCloud<PointT>::Ptr>> candidate_frames;
    for (const auto &key : closest_poses)
    {
        gtsam::Symbol symbol(key);
        int index = symbol.index();
        Pose3 pose = initial_values.at<Pose3>(key);
        pcl::PointCloud<PointT>::Ptr local_map(new pcl::PointCloud<PointT>);
        // 加载候选帧及其附近的点云
        for (int i = index - num_nearby_frames / 2; i <= index + num_nearby_frames / 2; ++i)
        {
            if (i < 0 || i >= initial_values.size())
            {
                continue;
            }
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            std::string cloud_file = cloud_directory + "key_point_frame/" + std::to_string(i) + ".pcd";
            pcl::io::loadPCDFile(cloud_file, *cloud);
            // 将点云从全局坐标系转换到候选位姿对应的帧上
            Pose3 nearby_pose = initial_values.at<Pose3>(X(i));
            Pose3 relative_pose = pose.between(nearby_pose);
            pcl::transformPointCloud(*cloud, *cloud, relative_pose.matrix());
            *local_map += *cloud;
        }
        // Downsample the local map
        pcl::PointCloud<PointT>::Ptr downsampled(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(local_map);
        sor.setLeafSize(0.4f, 0.4f, 0.4f);
        sor.filter(*downsampled);
        // std::cout << "localmap : " << downsampled->size() << " " << local_map->size() << std::endl;
        candidate_frames.emplace_back(index, pose, downsampled);
    }
    return candidate_frames;
}

//---------------------------------------------------------------------
// Compute ICP Covariance Matrix
//---------------------------------------------------------------------
Eigen::Matrix<float, 6, 6> CloudProcess::ComputeICPCovarianceMatrix(
    std::shared_ptr<open3d::geometry::PointCloud> &source,
    std::shared_ptr<open3d::geometry::PointCloud> &target,
    const Eigen::Matrix4d &transformation)
{
    using namespace open3d;
    double max_correspondence_distance_info = 2.0; // Set based on application needs.
    core::Tensor transformation_tensor = core::eigen_converter::EigenMatrixToTensor(transformation);
    t::geometry::PointCloud source_o3d_new = t::geometry::PointCloud::FromLegacy(*source);
    t::geometry::PointCloud target_o3d_new = t::geometry::PointCloud::FromLegacy(*target);
    core::Tensor information_matrix = t::pipelines::registration::GetInformationMatrix(
        source_o3d_new, target_o3d_new, max_correspondence_distance_info, transformation_tensor);
    Eigen::MatrixXd information_matrix_eigen = core::eigen_converter::TensorToEigenMatrixXd(information_matrix);
    Eigen::Matrix<float, 6, 6> icp_cov;
    if (information_matrix_eigen.rows() == 6 && information_matrix_eigen.cols() == 6)
    {
        icp_cov = information_matrix_eigen.inverse().cast<float>();
    }
    else
    {
        std::cerr << "[ERROR] Information matrix is not 6x6. Cannot compute covariance matrix." << std::endl;
        // Consider throwing an exception or using a default value rather than an identity matrix.
        icp_cov = Eigen::Matrix<float, 6, 6>::Identity();
    }
    return icp_cov;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
CloudProcess::extractLocalMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &global_map,
                              const Eigen::Matrix4d &initial_pose_matrix,
                              float radius)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZI>);
    // Convert Matrix4d to Affine3f for CropBox compatibility
    Eigen::Matrix3f rotation = initial_pose_matrix.block<3, 3>(0, 0).cast<float>();
    Eigen::Vector3f translation = initial_pose_matrix.block<3, 1>(0, 3).cast<float>();
    Eigen::Affine3f initial_pose;
    initial_pose.linear() = rotation;
    initial_pose.translation() = translation;
    // Define the bounding box dimensions
    Eigen::Vector4f min_pt(-radius, -radius, -std::numeric_limits<float>::infinity(), 1.0);
    Eigen::Vector4f max_pt(radius, radius, std::numeric_limits<float>::infinity(), 1.0);
    // Create the CropBox filter
    pcl::CropBox<pcl::PointXYZI> box_filter;
    box_filter.setInputCloud(global_map);
    box_filter.setMin(min_pt);
    box_filter.setMax(max_pt);
    box_filter.setTranslation(initial_pose.translation());
    box_filter.setRotation(initial_pose.rotation().eulerAngles(0, 1, 2));
    box_filter.filter(*local_map);
    return local_map;
}

//---------------------------------------------------------------------
// Perform ICP Registration
//---------------------------------------------------------------------
open3d::pipelines::registration::RegistrationResult CloudProcess::PerformICPRegistration(
    std::shared_ptr<open3d::geometry::PointCloud> &source,
    std::shared_ptr<open3d::geometry::PointCloud> &target,
    const Eigen::Matrix4d &init_pose,
    const open3d::pipelines::registration::ICPConvergenceCriteria &criteria,
    int method)
{
    using namespace open3d;
    pipelines::registration::RegistrationResult icp;
    double max_correspondence_distance = 2.0; // Set based on application requirements.

    switch (method)
    {
    case 0: // Point-to-Point ICP
        std::cout << "[INFO] Running point-to-point ICP..." << std::endl;
        icp = pipelines::registration::RegistrationICP(
            *source, *target, max_correspondence_distance, init_pose.cast<double>(),
            pipelines::registration::TransformationEstimationPointToPoint(), criteria);
        break;
    case 1: // Point-to-Plane ICP
        std::cout << "[INFO] Running point-to-plane ICP..." << std::endl;
        target->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 10));
        icp = pipelines::registration::RegistrationICP(
            *source, *target, max_correspondence_distance, init_pose.cast<double>(),
            pipelines::registration::TransformationEstimationPointToPlane(), criteria);
        break;
    case 2: // Generalized ICP
        std::cout << "[INFO] Running generalized ICP..." << std::endl;
        target->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 10));
        icp = pipelines::registration::RegistrationGeneralizedICP(
            *source, *target, max_correspondence_distance, init_pose.cast<double>(),
            pipelines::registration::TransformationEstimationForGeneralizedICP(), criteria);
        break;
    case 3: // Multi-Scale ICP
    {
        std::cout << "[INFO] Using multi-scale ICP!" << std::endl;
        target->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 10));
        auto source_tensor = open3d::t::geometry::PointCloud::FromLegacy(*source).VoxelDownSample(0.1);
        auto target_tensor = open3d::t::geometry::PointCloud::FromLegacy(*target).VoxelDownSample(0.1);
        open3d::core::Tensor init_matrix = open3d::core::eigen_converter::EigenMatrixToTensor(
            init_pose.cast<double>());
        std::vector<double> voxel_sizes = {0.5, 0.3, 0.2};
        std::vector<double> max_correspondence_distances = {5.0, 1.0, 0.5};
        std::vector<t::pipelines::registration::ICPConvergenceCriteria> criteria_list = {
            t::pipelines::registration::ICPConvergenceCriteria(0.01, 0.01, 15),
            t::pipelines::registration::ICPConvergenceCriteria(0.001, 0.001, 10),
            t::pipelines::registration::ICPConvergenceCriteria(0.000001, 0.000001, 5)};
        auto multi_icp = t::pipelines::registration::MultiScaleICP(
            source_tensor, target_tensor, voxel_sizes, criteria_list,
            max_correspondence_distances, init_matrix,
            t::pipelines::registration::TransformationEstimationPointToPlane());
        icp.transformation_ = open3d::core::eigen_converter::TensorToEigenMatrixXd(multi_icp.transformation_);
        icp.inlier_rmse_ = multi_icp.inlier_rmse_;
        icp.fitness_ = multi_icp.fitness_;
        break;
    }
    default:
        std::cout << "[ERROR] Evaluation error: Invalid ICP method type!" << std::endl;
        break;
    }
    return icp;
}

//---------------------------------------------------------------------
// Check ICP Registration Result
//---------------------------------------------------------------------
bool CloudProcess::CheckICPResult(double score, double overlap)
{
    if (score > 0.7 || overlap < 0.7 || score == 0.0)
    {
        std::cout << "[ERROR] ICP FAILED: score = " << score << ", overlap = " << overlap << std::endl;
        return false;
    }
    return true;
}
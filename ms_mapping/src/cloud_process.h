#ifndef SRC_POSE_SLAM_PRIOR_SRC_CLOUD_PROCESS_H_
#define SRC_POSE_SLAM_PRIOR_SRC_CLOUD_PROCESS_H_

#include <base_type.hpp>

class CloudProcess
{
public:
        CloudProcess() {};

        pcl::PointCloud<pcl::PointXYZI>::Ptr RemoveRangeCloud(
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Vector3i axis,
            Eigen::Vector3d threshold, std::string op);

        void CropGlobalCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &globalmap_ptr,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &temp_map,
                             Eigen::Matrix4d &pose, Eigen::Vector3d size);

        void AddNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                       pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);

        std::shared_ptr<open3d::geometry::PointCloud> GetO3dPointCloudFromPCL(
            pcl::PointCloud<pcl::PointXYZI> &pc);

        void FindNearKeyframes(std::vector<Measurement> &keyMeasures,
                               pcl::PointCloud<PointT>::Ptr &nearKeyframes,
                               const int &key, const int &searchNum);

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> CreateGlobalMap(const std::vector<Measurement> &measurements,
                                                                          const std::vector<int> &representative_indices,
                                                                          const std::string &cloud_directory,
                                                                          int frames_per_trajectory = 30);

        bool PerformICP(pcl::PointCloud<PointT>::Ptr &cureKeyframeCloud, pcl::PointCloud<PointT>::Ptr &trajectoryCloud,
                        Eigen::Matrix4d &final_trans, double &score, double &overlap);

        bool DoICPVirtualRelative3(std::vector<Measurement> &keyMeasures,
                                   pcl::PointCloud<PointT>::Ptr &targetKeyframeCloud,
                                   const std::string &mapDir,
                                   int loopKeyPre, int loopKeyCur,
                                   double &score, int type,
                                   Eigen::Matrix4d &trans);

        pcl::PointCloud<pcl::PointXYZI>::Ptr CreateLocalMapFromMeasurements(const std::vector<Measurement> &measurements,
                                                                            int history_frame_index, int curr_frame_index,
                                                                            const std::string &cloud_directory, int num_closest_poses = 100);

        Eigen::Matrix<float, 6, 6> ComputeICPCovarianceMatrix(
            std::shared_ptr<open3d::geometry::PointCloud> &source, std::shared_ptr<open3d::geometry::PointCloud> &target,
            const Eigen::Matrix4d &transformation);

        pcl::PointCloud<pcl::PointXYZI>::Ptr CreateLocalMap(const Eigen::Matrix4d &initial_pose_matrix,
                                                            const gtsam::Values &initial_values,
                                                            const std::string &cloud_directory,
                                                            int &closest_pose_index,
                                                            int num_closest_poses);

        std::vector<std::tuple<int, gtsam::Pose3, pcl::PointCloud<PointT>::Ptr>> CreateLocalMapMulti(
            const Eigen::Matrix4d &initial_pose_matrix, const gtsam::Values &initial_values,
            const std::string &cloud_directory, int &closest_pose_index, int num_closest_poses, int num_nearby_frames);

        pcl::PointCloud<pcl::PointXYZI>::Ptr extractLocalMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &global_map,
                                                             const Eigen::Matrix4d &initial_pose_matrix,
                                                             float radius);

        bool CheckICPResult(double score, double overlap);

        open3d::pipelines::registration::RegistrationResult PerformICPRegistration(
            std::shared_ptr<open3d::geometry::PointCloud> &source,
            std::shared_ptr<open3d::geometry::PointCloud> &target,
            const Eigen::Matrix4d &init_pose,
            const open3d::pipelines::registration::ICPConvergenceCriteria &criteria,
            int method);

        Eigen::Matrix6d icp_cov = Eigen::Matrix6d::Identity();
};

#endif // SRC_POSE_SLAM_PRIOR_SRC_CLOUD_PROCESS_H_

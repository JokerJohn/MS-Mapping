//
// Created by xchu on 8/3/2024.
//

#ifndef VOXEL_LIO_VOXELMAP_H
#define VOXEL_LIO_VOXELMAP_H

/*
#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <fast_lio/ikd-Tree/ikd_Tree.h>

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;
using PointCloudConstPtr = pcl::PointCloud<PointT>::ConstPtr;


class VoxelMap {
public:
    VoxelMap(double voxel_size, int min_points_per_voxel)
            : voxel_size_(voxel_size), min_points_per_voxel_(min_points_per_voxel) {}

    void Update(const PointCloudConstPtr &new_cloud, const Eigen::Matrix4d &pose) {
        // Transform the new point cloud to the global map coordinate system
        PointCloudPtr transformed_cloud(new PointCloud);
        pcl::transformPointCloud(*new_cloud, *transformed_cloud, pose);
        // Extract the local map
        PointCloudPtr local_map = ExtractLocalMap(transformed_cloud);
        // Create a k-d tree for the local map
        pcl::KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud(local_map);
        // Incrementally update the voxels affected by the new points
        for (const auto &point : transformed_cloud->points) {
            std::vector<int> indices;
            std::vector<float> distances;
            kdtree.radiusSearch(point, voxel_size_, indices, distances);
            for (const auto &idx : indices) {
                const auto &voxel_point = local_map->points[idx];
                Eigen::Vector3i voxel_index = GetVoxelIndex(voxel_point);
                auto it = voxel_map_.find(voxel_index);
                if (it == voxel_map_.end()) {
                    voxel_map_[voxel_index] = VoxelData(point, min_points_per_voxel_);
                } else {
                    it->second.Update(point);
                }
            }
        }
        // Remove voxels with too few points
        auto it = voxel_map_.begin();
        while (it != voxel_map_.end()) {
            if (it->second.NumPoints() < min_points_per_voxel_) {
                it = voxel_map_.erase(it);
            } else {
                ++it;
            }
        }
    }

    double ComputeEntropy() const {
        double entropy = 0.0;
        for (const auto &kv : voxel_map_) {
            const auto &voxel_data = kv.second;
            if (voxel_data.NumPoints() >= min_points_per_voxel_) {
                entropy += std::log(voxel_data.Covariance().determinant());
            }
        }
        return entropy;
    }

private:
    struct VoxelData {
        VoxelData(const PointT &point, int min_points)
                : num_points_(1), mean_(point.getVector3fMap().cast<double>()),
                  cov_(Eigen::Matrix3d::Zero()), min_points_(min_points) {}

        void Update(const PointT &point) {
            ++num_points_;
            const Eigen::Vector3d p = point.getVector3fMap().cast<double>();
            const Eigen::Vector3d delta = p - mean_;
            mean_ += delta / num_points_;
            cov_ += (delta * delta.transpose() - cov_) / num_points_;
        }

        int NumPoints() const { return num_points_; }

        const Eigen::Vector3d &Mean() const { return mean_; }

        const Eigen::Matrix3d &Covariance() const { return cov_; }

    private:
        int num_points_;
        Eigen::Vector3d mean_;
        Eigen::Matrix3d cov_;
        int min_points_;
    };

    PointCloudPtr ExtractLocalMap(const PointCloudConstPtr &new_cloud) const {
        PointCloudPtr local_map(new PointCloud);
        pcl::KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud(global_map_);
        std::vector<int> indices;
        std::vector<float> distances;
        kdtree.radiusSearch(new_cloud->points[0], local_map_radius_, indices, distances);
        pcl::copyPointCloud(*global_map_, indices, *local_map);
        return local_map;
    }

    Eigen::Vector3i GetVoxelIndex(const PointT &point) const {
        return Eigen::Vector3i(std::floor(point.x / voxel_size_),
                               std::floor(point.y / voxel_size_),
                               std::floor(point.z / voxel_size_));
    }

    double voxel_size_;
    int min_points_per_voxel_;
    double local_map_radius_ = 50.0;
    PointCloudPtr global_map_;
    std::unordered_map<Eigen::Vector3i, VoxelData, open3d::utility::hash_eigen<Eigen::Vector3i>> voxel_map_;
};

*/

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <unordered_map>
#include <vector>
#include <cmath>

#include <fast_lio/ikd-Tree/ikd_Tree.h>

using PointT = pcl::PointXYZINormal;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;
using PointCloudConstPtr = pcl::PointCloud<PointT>::ConstPtr;

// Hash function for Eigen::Vector3i
struct Eigen_Vector3i_hash {
    std::size_t operator()(const Eigen::Vector3i &vec) const {
        size_t seed = 0;
        for (int i = 0; i < 3; ++i) {
            seed ^= std::hash<int>()(vec[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

struct VoxelData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VoxelData() : num_points_(0), mean_(Eigen::Vector3d::Zero()), cov_(Eigen::Matrix3d::Zero()), min_points_(0) {}

    VoxelData(const PointT &point, int min_points);

    void Update(const PointT &point);

    void Merge(const VoxelData &other);

    int NumPoints() const;

    const Eigen::Vector3d &Mean() const;

    const Eigen::Matrix3d &Covariance() const;

private:
    int num_points_;
    Eigen::Vector3d mean_;
    Eigen::Matrix3d cov_;
    int min_points_;
};

class VoxelMap {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using VoxelHashMap = std::unordered_map<Eigen::Vector3i, VoxelData, Eigen_Vector3i_hash>;
    using PointVector = std::vector<PointT, Eigen::aligned_allocator<PointT>>;

    VoxelMap(double voxel_size, int min_points_per_voxel);

    static Eigen::Vector3i GetVoxelIndex(const PointT &point, double voxel_size);

    void ComputeLocalMapInfoGain(const BoxPointType &local_region,
                                 KD_TREE<PointT> &ikdtree,
                                 VoxelHashMap &local_voxel_map);

    double ComputeEntropyGain(const VoxelHashMap &local_voxel_map,
                              const VoxelHashMap &current_frame_voxel_map);

private:
    double voxel_size_;
    int min_points_per_voxel_;
};

// VoxelData struct implementation
inline VoxelData::VoxelData(const PointT &point, int min_points)
        : num_points_(1), mean_(point.getVector3fMap().cast<double>()),
          cov_(Eigen::Matrix3d::Zero()), min_points_(min_points) {}

inline void VoxelData::Update(const PointT &point) {
    ++num_points_;
    const Eigen::Vector3d p = point.getVector3fMap().cast<double>();
    const Eigen::Vector3d delta = p - mean_;
    mean_ += delta / num_points_;
    cov_ += (delta * delta.transpose() - cov_) / num_points_;
}

inline void VoxelData::Merge(const VoxelData &other) {
    double p1 = num_points_ / (num_points_ + other.num_points_);
    double p2 = other.num_points_ / (num_points_ + other.num_points_);
    mean_ = p1 * mean_ + p2 * other.mean_;
    cov_ = p1 * cov_ + p2 * other.cov_ + p1 * p2 * (mean_ - other.mean_) * (mean_ - other.mean_).transpose();
    num_points_ += other.num_points_;
}

inline int VoxelData::NumPoints() const { return num_points_; }

inline const Eigen::Vector3d &VoxelData::Mean() const { return mean_; }

inline const Eigen::Matrix3d &VoxelData::Covariance() const { return cov_; }

// VoxelMap class implementation
inline VoxelMap::VoxelMap(double voxel_size, int min_points_per_voxel)
        : voxel_size_(voxel_size), min_points_per_voxel_(min_points_per_voxel) {}

inline Eigen::Vector3i VoxelMap::GetVoxelIndex(const PointT &point, double voxel_size) {
    return Eigen::Vector3i(std::floor(point.x / voxel_size),
                           std::floor(point.y / voxel_size),
                           std::floor(point.z / voxel_size));
}

inline void VoxelMap::ComputeLocalMapInfoGain(const BoxPointType& local_region,
                                              KD_TREE<PointT>& ikdtree,
                                              VoxelHashMap& local_voxel_map)
{
    // Clear the local voxel map
    local_voxel_map.clear();
    // Extract the local point cloud within the bounding box
    std::vector<PointT> local_points;
    ikdtree.Search_by_range(local_region, local_points);
    // Voxelize the local point cloud
    for (const auto& point : local_points) {
        Eigen::Vector3i voxel_index = GetVoxelIndex(point, voxel_size_);
        auto it = local_voxel_map.find(voxel_index);
        if (it == local_voxel_map.end()) {
            local_voxel_map[voxel_index] = VoxelData(point, min_points_per_voxel_);
        } else {
            it->second.Update(point);
        }
    }
    // Remove voxels with too few points
    auto it = local_voxel_map.begin();
    while (it != local_voxel_map.end()) {
        if (it->second.NumPoints() < min_points_per_voxel_) {
            it = local_voxel_map.erase(it);
        } else {
            ++it;
        }
    }
}
inline double VoxelMap::ComputeEntropyGain(const VoxelHashMap& local_voxel_map,
                                           const VoxelHashMap& current_frame_voxel_map)
{
    // Compute the initial entropy of the local voxel map
    double initial_entropy = 0.0;
    for (const auto& kv : local_voxel_map) {
        const auto& voxel_data = kv.second;
        if (voxel_data.NumPoints() >= min_points_per_voxel_) {
            double p = voxel_data.NumPoints() / (voxel_data.NumPoints() + 1e-6);
            initial_entropy += p * std::log(voxel_data.Covariance().determinant());
        }
    }
    // Merge the current frame voxel map into the local voxel map
    auto merged_voxel_map = local_voxel_map;
    for (const auto& kv : current_frame_voxel_map) {
        const auto& voxel_index = kv.first;
        const auto& voxel_data = kv.second;
        auto it = merged_voxel_map.find(voxel_index);
        if (it == merged_voxel_map.end()) {
            merged_voxel_map[voxel_index] = voxel_data;
        } else {
            it->second.Merge(voxel_data);
        }
    }
    // Compute the updated entropy of the merged voxel map
    double updated_entropy = 0.0;
    for (const auto& kv : merged_voxel_map) {
        const auto& voxel_data = kv.second;
        if (voxel_data.NumPoints() >= min_points_per_voxel_) {
            double p = voxel_data.NumPoints() / (voxel_data.NumPoints() + 1e-6);
            updated_entropy += p * std::log(voxel_data.Covariance().determinant());
        }
    }
    // Compute the information gain
    double information_gain = initial_entropy - updated_entropy;
    return information_gain;
}
#endif //VOXEL_LIO_VOXELMAP_H

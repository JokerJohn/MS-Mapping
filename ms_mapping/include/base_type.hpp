
#ifndef IMAGING_LIDAR_PLACE_RECOGNITION_PGO_INCLUDE_BASE_TYPE_H_
#define IMAGING_LIDAR_PLACE_RECOGNITION_PGO_INCLUDE_BASE_TYPE_H_

// pcl
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/search/impl/search.hpp>

// yaml-cpp
#include <yaml-cpp/yaml.h>
// gtsam
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/inferenceExceptions.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <open3d/Open3D.h>


// ros
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// c++
#include <condition_variable>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <string>
#include <thread>
#include <utility>
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <algorithm>
#include <memory>

// opencv
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


// #include <gflags/gflags.h>
// #include <glog/logging.h>

// #include <fmt/core.h>
// #include <fmt/format.h>
// #include <fmt/ostream.h>

#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

using namespace std;
using namespace Eigen;
typedef pcl::PointXYZI PointT;

using gtsam::symbol_shorthand::G; 
using gtsam::symbol_shorthand::X; 
using number_t = double;

enum BA_WEIGHT {
    CONSTANT, LOCAL, LOCAL_GLOBAL
};
enum BA_LOSS {
    NONE, LCONSTANT, SCALAR_DEPEND, CHI2
};

/// A std::map that uses Eigen::aligned_allocator so that the
/// contained types may be fixed-size Eigen values.
template<typename Key, typename T>
using eigen_aligned_std_map =
std::map<Key, T, std::less<Key>,  Eigen::aligned_allocator<std::pair<Key const, T>>>;

/// A std::unordered_map that uses Eigen::aligned_allocator so that the
/// contained types may be fixed-size Eigen values.
template<typename Key, typename T>
using eigen_aligned_std_unordered_map =
std::unordered_map<Key, T, std::hash<Key>, std::equal_to<Key>,
        Eigen::aligned_allocator<std::pair<Key const, T>>>;

/// A std::vector that uses Eigen::aligned_allocator so that the contained
/// types may be fixed-size Eigen values.
template<typename T>
using eigen_aligned_std_vector = std::vector<T, Eigen::aligned_allocator<T>>;


/// The empty column vector (zero rows, one column), templated on scalar type.
template<typename Scalar>
using Vector0 = Eigen::Matrix<Scalar, 0, 1>;

/// A column vector of size 1 (that is, a scalar), templated on scalar type.
// template <typename Scalar>
// using Vector1 = Eigen::Matrix<Scalar, 1, 1>;
using Vector1d = Eigen::Matrix<double, 1, 1>;
using Vector1f = Eigen::Matrix<float, 1, 1>;
using Vector1i = Eigen::Matrix<int, 1, 1>;
using Vector1 = Eigen::Matrix<number_t, 1, 1>;

/// A column vector of size 2, templated on scalar type.
// template <typename Scalar>
// using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
// using Vector2d = Eigen::Vector<double, >
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Vector2 = Eigen::Matrix<number_t, 2, 1>;

/// A column vector of size 3, templated on scalar type.
// template <typename Scalar>
// using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Eigen::Vector3d;
using Eigen::Vector3f;
using Eigen::Vector3i;
using Vector3 = Eigen::Matrix<number_t, 3, 1>;

/// A column vector of size 4, templated on scalar type.
// template <typename Scalar>
// using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
using Eigen::Vector4d;
using Eigen::Vector4f;
using Eigen::Vector4i;
using Vector4 = Eigen::Matrix<number_t, 4, 1>;

/// A column vector of size 6.
// template <typename Scalar>
// using Vector5 = Eigen::Matrix<Scalar, 5, 1>;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Vector5f = Eigen::Matrix<float, 5, 1>;
using Vector5i = Eigen::Matrix<int, 5, 1>;
using Vector5 = Eigen::Matrix<number_t, 5, 1>;

/// A column vector of size 6.
// template <typename Scalar>
// using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector6f = Eigen::Matrix<float, 6, 1>;
using Vector6i = Eigen::Matrix<int, 6, 1>;
using Vector6 = Eigen::Matrix<number_t, 6, 1>;

/// A column vector templated on the number of rows.
// template <typename Scalar, int Rows>
// using Vector = Eigen::Matrix<Scalar, Rows, 1>;

/// A column vector of any size, templated on scalar type.
// template <typename Scalar>
// using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using VectorXf = Eigen::VectorXf;
using VectorXd = Eigen::VectorXd;
// using VectorX = Eigen::VectorX<number_t>;
using VectorX = Eigen::Matrix<number_t, Eigen::Dynamic, 1>;

/// A vector of dynamic size templated on scalar type, up to a maximum of 6
/// elements.
// template <typename Scalar>
// using VectorUpTo6 = Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, 6, 1>;

/// A row vector of size 2, templated on scalar type.
// template <typename Scalar>
// using RowVector2 = Eigen::Matrix<Scalar, 1, 2>;
using Eigen::RowVector2d;
using Eigen::RowVector2f;
using Eigen::RowVector2i;
using RowVector2 = Eigen::Matrix<number_t, 1, 2>;

/// A row vector of size 3, templated on scalar type.
// template <typename Scalar>
// using RowVector3 = Eigen::Matrix<Scalar, 1, 3>;
using Eigen::RowVector3d;
using Eigen::RowVector3f;
using Eigen::RowVector3i;
using RowVector3 = Eigen::Matrix<number_t, 1, 3>;

/// A row vector of size 4, templated on scalar type.
// template <typename Scalar>
// using RowVector4 = Eigen::Matrix<Scalar, 1, 4>;
using Eigen::RowVector4d;
using Eigen::RowVector4f;
using Eigen::RowVector4i;
using RowVector4 = Eigen::Matrix<number_t, 1, 4>;

/// A row vector of size 6.
// template <typename Scalar>
// using RowVector6 = Eigen::Matrix<Scalar, 1, 6>;
using RowVector6d = Eigen::Matrix<double, 1, 6>;
using RowVector6f = Eigen::Matrix<float, 1, 6>;
using RowVector6i = Eigen::Matrix<int, 1, 6>;
using RowVector6 = Eigen::Matrix<number_t, 1, 6>;

/// A row vector templated on the number of columns.
// template <typename Scalar, int Cols>
// using RowVector = Eigen::Matrix<Scalar, 1, Cols>;

/// A row vector of any size, templated on scalar type.
// template <typename Scalar>
// using RowVectorX = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>;
using RowVectorXf = Eigen::RowVectorXf;
using RowVectorXd = Eigen::RowVectorXd;
// using RowVectorX = Eigen::RowVectorX<number_t>;
using RowVectorX = Eigen::Matrix<number_t, 1, Eigen::Dynamic>;

/// A matrix of 2 rows and 2 columns, templated on scalar type.
// template <typename Scalar>
// using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;
using Eigen::Matrix2d;
using Eigen::Matrix2f;
using Eigen::Matrix2i;
using Matrix2 = Eigen::Matrix<number_t, 2, 2>;

/// A matrix of 3 rows and 3 columns, templated on scalar type.
// template <typename Scalar>
// using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Matrix3i;
using Matrix3 = Eigen::Matrix<number_t, 3, 3>;

/// A matrix of 4 rows and 4 columns, templated on scalar type.
// template <typename Scalar>
// using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
using Eigen::Matrix4d;
using Eigen::Matrix4f;
using Eigen::Matrix4i;
using Matrix4 = Eigen::Matrix<number_t, 4, 4>;

/// A matrix of 6 rows and 6 columns, templated on scalar type.
// template <typename Scalar>
// using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix6f = Eigen::Matrix<float, 6, 6>;
using Matrix6i = Eigen::Matrix<int, 6, 6>;
using Matrix6 = Eigen::Matrix<number_t, 6, 6>;

/// A matrix of 2 rows, dynamic columns, templated on scalar type.
template<typename Scalar>
using Matrix2X = Eigen::Matrix<Scalar, 2, Eigen::Dynamic>;

/// A matrix of 3 rows, dynamic columns, templated on scalar type.
template<typename Scalar>
using Matrix3X = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;

/// A matrix of 4 rows, dynamic columns, templated on scalar type.
template<typename Scalar>
using Matrix4X = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>;

/// A matrix of 6 rows, dynamic columns, templated on scalar type.
template<typename Scalar>
using Matrix6X = Eigen::Matrix<Scalar, 6, Eigen::Dynamic>;

/// A matrix of dynamic size, templated on scalar type.
template<typename Scalar>
using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
using MatrixXd = MatrixX<double>;

/// A matrix of dynamic size templated on scalar type, up to a maximum of 6 rows
/// and 6 columns. Rectangular matrices, with different number of rows and
/// columns, are allowed.
template<typename Scalar>
using MatrixUpTo6 =
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 6>;

/// A quaternion templated on scalar type.
// template <typename Scalar>
// using Quaternion = Eigen::Quaternion<Scalar>;
using Eigen::Quaterniond;
using Eigen::Quaternionf;
using Quaternion = Eigen::Quaternion<number_t>;

/// An AngleAxis templated on scalar type.
// template <typename Scalar>
// using AngleAxis = Eigen::AngleAxis<Scalar>;
using Eigen::AngleAxisd;
using Eigen::AngleAxisf;
using AngleAxis = Eigen::AngleAxis<number_t>;

/// An Isometry templated on scalar type.
// template <typename Scalar>
// using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
using Eigen::Isometry3d;
using Eigen::Isometry3f;
using Isometry3 = Eigen::Transform<number_t, 3, Eigen::Isometry>;

/// A translation in 3D templated on scalar type.
template<typename Scalar>
using Translation3 = Eigen::Translation<Scalar, 3>;

struct Pose6D {
    double x, y, z, roll, pitch, yaw;
    double vx, vy, vz;
    double bax, bay, baz;
    double bgx, bgy, bgz;
    bool valid;
    Eigen::Matrix<double, 6, 1> cov;
    Eigen::Matrix<double, 6, 6> pose_cov;
    Eigen::Quaterniond quaternion;

    Pose6D() : x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0),
               vx(0.0), vy(0.0), vz(0.0),
               bax(0.0), bay(0.0), baz(0.0),
               bgx(0.0), bgy(0.0), bgz(0.0),
               valid(true), quaternion(1.0, 0.0, 0.0, 0.0),
               cov(Eigen::Matrix<double, 6, 1>::Identity()) {
        pose_cov.setIdentity();
    }

    Pose6D(double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
            : x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw),
              quaternion(Eigen::AngleAxisd(_yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(_pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(_roll, Eigen::Vector3d::UnitX())) {}

    void setBias(double _bax, double _bay, double _baz, double _bgx, double _bgy, double _bgz) {
        bax = _bax;
        bay = _bay;
        baz = _baz;
        bgx = _bgx;
        bgy = _bgy;
        bgz = _bgz;
    }

    void setRPY(double _roll, double _pitch, double _yaw) {
        roll = _roll;
        pitch = _pitch;
        yaw = _yaw;
        quaternion = Eigen::Quaterniond(
                Eigen::AngleAxisd(_yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(_pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(_roll, Eigen::Vector3d::UnitX()));
    }

    Pose6D &operator=(const Pose6D &s) {
        if (this == &s) return *this;
        x = s.x;
        y = s.y;
        z = s.z;
        roll = s.roll;
        pitch = s.pitch;
        yaw = s.yaw;
        vx = s.vx;
        vy = s.vy;
        vz = s.vz;
        bax = s.bax;
        bay = s.bay;
        baz = s.baz;
        bgx = s.bgx;
        bgy = s.bgy;
        bgz = s.bgz;
        quaternion = s.quaternion;
        cov = s.cov;
        pose_cov = s.pose_cov;
        valid = s.valid;
        return *this;
    }
};

struct Measurement {
    double lidar_time, odom_time, distance, global_score;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar;
    nav_msgs::Odometry odom, imu_odom, updated_odom, gps;
    bool has_gps, has_imu;
    std::deque<sensor_msgs::Imu> imu_deque;
    Pose6D key_pose, updated_pose, global_pose;
    bool isKey;
    Eigen::Matrix<double, 10, 1> time_data;
    Eigen::MatrixXd global_cov;     // 协方差（假设为某种协方差类型）
    Matrix6d pose_covariance;  // Added pose covariance
    bool has_gl;

    Measurement() : lidar_time(0.0), odom_time(0.0), distance(0.0), global_score(0), isKey(false),
                    lidar(new pcl::PointCloud<pcl::PointXYZI>()),
                    has_gps(false), has_imu(false) {
        time_data = Eigen::Matrix<double, 10, 1>::Zero();
        pose_covariance.setIdentity();
        has_gl = false;
    }
};

struct State {
    gtsam::Pose3 pose;
    gtsam::Velocity3 vel;
    Matrix6 cov;

    State() {
        pose = gtsam::Pose3::Identity();
        vel = gtsam::Velocity3::Zero();
        cov = Matrix6::Identity();
    };

};

struct ICPResult {
    double rmse;
    double overlap;
    bool degenerate;
    Matrix6 cov;
    bool issuccess;
    int iterate_number;
    gtsam::Pose3 trans;

    ICPResult() {
        rmse = 0.0;
        overlap = 0.0;
        degenerate = false;
        issuccess = false;
        cov = Matrix6::Identity();
        iterate_number = 0;
        trans = gtsam::Pose3::Identity();
    }

};


extern bool useGPS;
extern bool useScancontext;
extern bool useLoopCloser;
extern bool saveResultBag;
extern bool saveResultBodyFrame;
extern int method;
extern int imuFactorType;

extern bool useImuFrame;
extern bool useGlobalPrior;
extern int baseline;
extern bool useMultiMode;

extern bool saveKeyFrame;
extern bool saveLoopdata;
extern bool useKeyframe;
extern bool useFixedCov;
extern bool useRawCloud;
extern bool useImuFactor;
extern bool useCompressedImage;

extern double gpsFrequence;
extern double gpsCovThreshold;
extern double keyframeMeterGap;
extern double keyframeDegGap;
extern double keyframeRadGap;
extern double pixelDis;
extern vector<double> origin_lla;

extern double scDistThres;
extern double scMaximumRadius;
extern double historyKeyframeSearchRadius;
extern double historyKeyframeSearchTimeDiff;
extern double loopFitnessScoreThreshold;

extern int historyKeyframeSearchNum;
extern int accumulatedFactorSize;
extern double loopClosureFrequency;
extern int lidar_type;
extern double mea_noise;


extern double filterDis;
extern double LOOP_Z_OFFSET;
extern int filterNodeNum;
extern bool useRPGO;
extern int SKIP_FRAMES;
extern int GPS_SKIP_NUMBER;
extern std::string configDirectory;
extern std::string saveDirectory;
extern std::string mapDirectory;
extern std::string sequence;
extern std::string odom_link;
extern std::string gps_topic;
extern std::string imu_topic;
extern std::string left_image_topic;
extern std::string right_image_topic;

extern double historyKeyframeSearchRadius;
extern double historyKeyframeSearchTimeDiff;
extern int historyKeyframeSearchNum;
extern double loopFitnessScoreThreshold;
extern int accumulatedFactorSize;

extern double scan_filter_size;
extern double map_viewer_size;
extern double map_saved_size;

extern double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
extern vector<double> b_gyr_cov_n;
extern vector<double> b_acc_cov_n;

extern vector<double> extrinT;
extern vector<double> extrinR;
extern Eigen::Quaterniond q_body_sensor;
extern Eigen::Matrix3d rot_body_sensor;
extern Eigen::Vector3d t_body_sensor;

extern vector<double> camera_matrix0_vec;
extern vector<double> distCoeffs0_vec;
extern Eigen::Vector4d distCoeffs0, distCoeffs1;
extern vector<double> rot_camera2imu0_vec;
extern vector<double> trans_camera2imu0_vec;
extern Eigen::Matrix3d camera_matrix0;
extern Eigen::Quaterniond q_camera0_body;
extern Eigen::Matrix3d rot_camera0_body;
extern Eigen::Vector3d t_camera0_body;

extern vector<double> camera_matrix1_vec;
extern vector<double> distCoeffs1_vec;
extern vector<double> rot_camera2imu1_vec;
extern vector<double> trans_camera2imu1_vec;
extern Eigen::Matrix3d camera_matrix1;
extern Eigen::Quaterniond q_camera1_body;
extern Eigen::Matrix3d rot_camera1_body;
extern Eigen::Vector3d t_camera1_body;

std::string toString(Pose6D &pose);

extern Pose6D getOdom(nav_msgs::Odometry _odom);

extern Pose6D diffTransformation(const Pose6D &_p1, const Pose6D &_p2);

extern Eigen::Matrix4d Pose6D2Matrix(const Pose6D &p);

extern Pose6D Matrix2Pose6D(const Eigen::Matrix4d &matrix);

extern Pose6D Pose6D2Body(const Pose6D &p, const Eigen::Matrix4d &matrix);

extern Pose6D Isometry3d2Pose6D(const Eigen::Isometry3d &matrix);

extern Eigen::Isometry3d Pose6D2sometry3d(const Pose6D &p);

extern Eigen::Isometry3d OdomToIsometry3d(const nav_msgs::Odometry &pose_geo);

extern Eigen::Isometry3d GeoposeToIsometry3d(
        const geometry_msgs::PoseWithCovarianceStamped &pose_geo);

extern Pose6D GtsamPose2Pose6D(gtsam::Pose3 pose);

extern Eigen::Affine3f Pose6dToAffine3f(Pose6D pose);

extern gtsam::Pose3 Pose6dTogtsamPose3(Pose6D pose);

extern gtsam::Pose3 Matrix4dToPose3(const Eigen::Matrix4d &matrix) ;

extern Eigen::Matrix4d GeoposeToMatrix4d(
        const geometry_msgs::PoseWithCovarianceStamped &pose_geo);

pcl::PointCloud<PointT>::Ptr TransformPointCloud(
        const pcl::PointCloud<PointT>::Ptr &cloudIn, const Pose6D &tf);

pcl::PointCloud<PointT>::Ptr TransformPointCloud(
        const pcl::PointCloud<PointT>::Ptr &cloudIn,
        const Eigen::Matrix4d &transformIn);

pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc(
        const std::vector<Pose6D> vectorPose6d);

pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc2d(
        const std::vector<Pose6D> vectorPose6d);

void LoadYamlFile(string path);

void LoadRosParams(ros::NodeHandle &nh);

template<typename T>
T readParam(ros::NodeHandle &n, std::string name) {
    T ans;
    if (n.getParam(name, ans)) {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    } else {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher &thisPub,
                                      const T &thisCloud, ros::Time thisStamp,
                                      std::string thisFrame) {
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0) thisPub.publish(tempCloud);
    return tempCloud;
}

#endif
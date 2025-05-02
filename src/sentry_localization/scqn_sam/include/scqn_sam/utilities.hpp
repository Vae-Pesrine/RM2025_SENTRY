#pragma once

// common
#include <string>
#include <cstdlib>
#include <sys/stat.h>
#include <unistd.h>

// ros
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h> // quaternion to euler
#include <tf/LinearMath/Matrix3x3.h>  // quaternion to euler
#include <tf/transform_datatypes.h>   // rpy --> quaternion
#include <tf_conversions/tf_eigen.h>  // tf <-> eigen
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// pcl
#include <pcl/point_types.h>                 //pt
#include <pcl/point_cloud.h>                 //cloud
#include <pcl/conversions.h>                 //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h> //voxelgrid

// eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)

// gtsam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

using PointType = pcl::PointXYZI;

/**
 * @brief Downsample point cloud using voxel grid filter.
 * @tparam CloudT Point cloud type (can be pointer or reference).
 * @param cloud_in Input point cloud.
 * @param voxel_res Voxel grid resolution (meters).
 * @return Downsampled point cloud pointer.
 */
inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType> &pcd_in,
    const float voxel_res)
{
    pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    pcl::PointCloud<PointType>::Ptr pcd_in_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
    pcd_in_ptr->reserve(pcd_in.size());
    pcd_out->reserve(pcd_in.size());
    *pcd_in_ptr = pcd_in;
    voxelgrid.setInputCloud(pcd_in_ptr);
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType>::Ptr &pcd_in,
    const float voxel_res)
{
    pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
    pcd_out->reserve(pcd_in->size());
    voxelgrid.setInputCloud(pcd_in);
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

/**
 * @brief Convert 4x4 Eigen pose matrix to gtsam::Pose3.
 * @param pose_eig_in Input Eigen pose matrix.
 * @return gtsam::Pose3 object.
 */
inline gtsam::Pose3 poseEigToGtsamPose(
    const Eigen::Matrix4d &pose_eig_in
) {
    double r, p, y;
    tf::Matrix3x3 mat;
    tf::matrixEigenToTF(pose_eig_in.block<3, 3>(0, 0), mat);
    mat.getRPY(r, p, y);

    return gtsam::Pose3(gtsam::Rot3::RzRyRx(r, p, y),
                        gtsam::Point3(pose_eig_in(0, 3), pose_eig_in(1, 3), pose_eig_in(2, 3)));
}

/**
 * @brief Convert gtsam::Pose3 to 4x4 Eigen pose matrix.
 * @param gtsam_pose_in Input gtsam::Pose3.
 * @return Eigen 4x4 pose matrix.
 */
inline Eigen::Matrix4d gtsamPoseToPoseEig(
    const gtsam::Pose3 &gtsam_pose_in
) {
    Eigen::Matrix4d pose_eig_out = Eigen::Matrix4d::Identity();
    tf::Quaternion quat = tf::createQuaternionFromRPY(gtsam_pose_in.rotation().roll(),
                                                      gtsam_pose_in.rotation().pitch(),
                                                      gtsam_pose_in.rotation().yaw());
    tf::Matrix3x3 mat(quat);
    Eigen::Matrix3d tmp_rot_mat;
    tf::matrixTFToEigen(mat, tmp_rot_mat);
    pose_eig_out.block<3, 3>(0, 0) = tmp_rot_mat;
    pose_eig_out(0, 3) = gtsam_pose_in.translation().x();
    pose_eig_out(1, 3) = gtsam_pose_in.translation().y();
    pose_eig_out(2, 3) = gtsam_pose_in.translation().z();

    return pose_eig_out;
}

/**
 * @brief Convert 4x4 Eigen pose matrix to geometry_msgs::PoseStamped.
 * @param pose_eig_in Input Eigen pose matrix.
 * @param frame_id ROS frame id.
 * @return geometry_msgs::PoseStamped message.
 */
inline geometry_msgs::PoseStamped poseEigToPoseStamped(
    const Eigen::Matrix4d &pose_eig_in,
    std::string frame_id = "map"
) {
    Eigen::Quaterniond quat(pose_eig_in.block<3, 3>(0, 0));
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = pose_eig_in(0, 3);
    pose.pose.position.y = pose_eig_in(1, 3);
    pose.pose.position.z = pose_eig_in(2, 3);
    pose.pose.orientation.w = quat.w();
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();

    return pose;
}

/**
 * @brief Convert 4x4 Eigen pose matrix to tf::Transform for ROS broadcaster.
 * @param pose Input Eigen pose matrix.
 * @return tf::Transform object.
 */
inline tf::Transform poseEigToROSTf(
    const Eigen::Matrix4d &pose
) {
    tf::Matrix3x3 tf_rot;
    tf::matrixEigenToTF(pose.block<3, 3>(0, 0), tf_rot);
    tf::Vector3 tf_trans(pose(0, 3), pose(1, 3), pose(2, 3));
    tf::Transform transform;
    transform.setOrigin(tf_trans);
    transform.setBasis(tf_rot);

    return transform;
}

/**
 * @brief Convert geometry_msgs::PoseStamped to tf::Transform.
 * @param pose Input geometry_msgs::PoseStamped.
 * @return tf::Transform object.
 */
inline tf::Transform poseStampedToROSTf(
    const geometry_msgs::PoseStamped &pose
) {
    const auto &p = pose.pose.position;
    const auto &q = pose.pose.orientation;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(p.x, p.y, p.z));
    transform.setRotation(tf::Quaternion(q.x, q.y, q.z, q.w));

    return transform;
}

/**
 * @brief Convert gtsam::Pose3 to geometry_msgs::PoseStamped.
 * @param gtsam_pose_in Input gtsam::Pose3.
 * @param frame_id ROS frame id.
 * @return geometry_msgs::PoseStamped message.
 */
inline geometry_msgs::PoseStamped gtsamPoseToPoseStamped(
    const gtsam::Pose3 &gtsam_pose_in,
    std::string frame_id = "map"
) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = gtsam_pose_in.translation().x();
    pose.pose.position.y = gtsam_pose_in.translation().y();
    pose.pose.position.z = gtsam_pose_in.translation().z();
    
    const auto &rot = gtsam_pose_in.rotation().matrix();
    Eigen::Quaterniond quat(rot);
    pose.pose.orientation.w = quat.w();
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();

    return pose;
}

/**
 * @brief Convert pcl point cloud to sensor_msgs::PointCloud2.
 * @tparam T Point type.
 * @param cloud Input point cloud.
 * @param frame_id ROS frame id.
 * @return sensor_msgs::PointCloud2 message.
 */
template<typename T>
inline sensor_msgs::PointCloud2 pclToPclRos(
    const pcl::PointCloud<T> &cloud,
    std::string frame_id = "map"
) {
    sensor_msgs::PointCloud2 cloud_ros;
    pcl::toROSMsg(cloud, cloud_ros);
    cloud_ros.header.frame_id = frame_id;

    return cloud_ros;
}

/**
 * @brief Apply transformation to pcl point cloud.
 * @tparam T Point type.
 * @param cloud_in Input point cloud.
 * @param pose_tf 4x4 transformation matrix.
 * @return Transformed point cloud.
 */
template<typename T>
inline pcl::PointCloud<T> transformPcd(
    const pcl::PointCloud<T> &cloud_in,
    const Eigen::Matrix4d &pose_tf
) {
    if (cloud_in.empty()){
        return pcl::PointCloud<T>();
    }

    pcl::PointCloud<T> pcl_out;
    pcl::transformPointCloud(cloud_in, pcl_out, pose_tf);

    return pcl_out;
}

inline bool dirExists(
    const std::string &path
) {
    struct stat info;
    return stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
}

inline void removeDir(
    const std::string &path
) {
    std::string cmd = "sudo rm -rf" + path;
    std::system(cmd.c_str());
}

inline void makeDir(
    const std::string &path
) {
    std::string cmd = "sudo mkdir -p" + path;
    std::system(cmd.c_str());
}

constexpr const char* RESET = "\033[0m\n";
constexpr const char* BLACK = "\033[30m";
constexpr const char* RED = "\033[31m";
constexpr const char* GREEN = "\033[32m";
constexpr const char* YELLOW = "\033[33m";
constexpr const char* BLUE = "\033[34m";
constexpr const char* PURPLE_RED = "\033[35m";
constexpr const char* WHITE = "\033[37m"; 
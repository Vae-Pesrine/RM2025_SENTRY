#pragma once

// usr
#include "utilities.hpp"

/**
 * @brief 存储点云、位姿、时间戳、索引等信息的结构体
 */
struct PosePcdStamped 
{
    pcl::PointCloud<PointType> pcd_;
    Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    double timestamp_ = 0.0;
    int index_ = 0;
    bool processed_ = false;

    PosePcdStamped() = default;
    explicit PosePcdStamped(const nav_msgs::Odometry& odom_in,
                          const sensor_msgs::PointCloud2& pcd_in,
                          int index_in);
};

/**
 * @brief 存储点云和位姿（含校正位姿、索引、处理标志）的结构体
 */
struct PosePcd 
{
    pcl::PointCloud<PointType> pcd_;
    Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    int index_ = 0;
    bool processed_ = false;

    PosePcd() = default;
    explicit PosePcd(const nav_msgs::Odometry& odom_in,
                     const sensor_msgs::PointCloud2& pcd_in,
                     int index_in);
};

/**
 * @brief 存储点云和位姿（无校正位姿）的结构体
 */
struct PosePcdReduced 
{
    pcl::PointCloud<PointType> pcd_;
    Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
    int index_ = 0;

    PosePcdReduced() = default;
    explicit PosePcdReduced(const geometry_msgs::PoseStamped& pose_in,
                            const sensor_msgs::PointCloud2& pcd_in,
                            int index_in);
};

// ================== 构造函数实现 ==================

inline PosePcd::PosePcd(
    const nav_msgs::Odometry& odom_in,
    const sensor_msgs::PointCloud2& pcd_in,
    int index_in
) {
    tf::Quaternion q(odom_in.pose.pose.orientation.x,
                     odom_in.pose.pose.orientation.y,
                     odom_in.pose.pose.orientation.z,
                     odom_in.pose.pose.orientation.w);
    tf::Matrix3x3 rot_matrix_tf(q);
    Eigen::Matrix3d rot_matrix_eigen;
    tf::matrixTFToEigen(rot_matrix_tf, rot_matrix_eigen);

    pose_eig_.block<3, 3>(0, 0) = rot_matrix_eigen;
    pose_eig_(0, 3) = odom_in.pose.pose.position.x;
    pose_eig_(1, 3) = odom_in.pose.pose.position.y;
    pose_eig_(2, 3) = odom_in.pose.pose.position.z;
    pose_corrected_eig_ = pose_eig_;

    pcl::PointCloud<PointType> cloud_tmp;
    pcl::fromROSMsg(pcd_in, cloud_tmp);
    pcd_ = transformPcd(cloud_tmp, pose_eig_.inverse());

    index_ = index_in;
}

inline PosePcdReduced::PosePcdReduced(
    const geometry_msgs::PoseStamped& pose_in,
    const sensor_msgs::PointCloud2& pcd_in,
    int index_in
) {
    tf::Quaternion q(pose_in.pose.orientation.x,
                     pose_in.pose.orientation.y,
                     pose_in.pose.orientation.z,
                     pose_in.pose.orientation.w);
    tf::Matrix3x3 rot_matrix_tf(q);
    Eigen::Matrix3d rot_matrix_eigen;
    tf::matrixTFToEigen(rot_matrix_tf, rot_matrix_eigen);

    pose_eig_.block<3, 3>(0, 0) = rot_matrix_eigen;
    pose_eig_(0, 3) = pose_in.pose.position.x;
    pose_eig_(1, 3) = pose_in.pose.position.y;
    pose_eig_(2, 3) = pose_in.pose.position.z;
    pcl::fromROSMsg(pcd_in, pcd_);
    index_ = index_in;
}

inline PosePcdStamped::PosePcdStamped(
    const nav_msgs::Odometry& odom_in,
    const sensor_msgs::PointCloud2& pcd_in,
    int index_in
) {
    tf::Quaternion q(odom_in.pose.pose.orientation.x,
                     odom_in.pose.pose.orientation.y,
                     odom_in.pose.pose.orientation.z,
                     odom_in.pose.pose.orientation.w);
    tf::Matrix3x3 rot_mat_tf(q);
    Eigen::Matrix3d rot_mat_eigen;
    tf::matrixTFToEigen(rot_mat_tf, rot_mat_eigen);
    pose_eig_.block<3, 3>(0, 0) = rot_mat_eigen;
    pose_eig_(0, 3) = odom_in.pose.pose.position.x;
    pose_eig_(1, 3) = odom_in.pose.pose.position.y;
    pose_eig_(2, 3) = odom_in.pose.pose.position.z;
    pose_corrected_eig_ = pose_eig_;
    pcl::PointCloud<PointType> tmp_pcd;
    pcl::fromROSMsg(pcd_in, tmp_pcd);
    pcd_ = transformPcd(tmp_pcd, pose_eig_.inverse());
    timestamp_ = odom_in.header.stamp.toSec();
    index_ = index_in;
}
#pragma once

// common
#include <string>
#include <memory>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//tf
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// small gicp
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>

// gpu bbs3d
#include <gpu_bbs3d/bbs3d.cuh>


class Relocalization: ros::NodeHandle
{
public: 
    Relocalization();
    ~Relocalization();

private:

    void setInitialTransformation();

    void loadGlobalMap(const std::string& file_name);

    void runRegistration(const ros::WallTimerEvent& event);

    void publishTransform(const ros::WallTimerEvent& event);

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    int getNearestImuIndex(const std::vector<sensor_msgs::Imu> &imu_buffer,
                           const ros::Time &stamp);

    void registerPcdCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle pr_nh_;
    ros::Subscriber sub_pcd_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_initialpose_;
    ros::Publisher pub_pcd_;

    bool debug_en_;
    bool pub_prior_pcd_en_;
    
    double x_, y_, z_;
    double yaw_, pitch_, roll_;

    double registration_frequency_;
    double pose_update_frequency_;

    double transform_tolerance_;
    double align_time_begin_;
    double align_time_end_;

    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string lidar_frame_;
    std::string prior_pcd_file_;
    std::string pcd_in_topic_;
    std::string imu_in_topic_;
    std::string initialpose_in_topic_;

    ros::Time last_scan_time_;
    Eigen::Isometry3d initial_transform_;
    Eigen::Isometry3d result_t_;
    Eigen::Isometry3d prior_result_t_;

    ros::WallTimer register_timer_;  
    ros::WallTimer transform_timer_;

    // msg buffer
    sensor_msgs::PointCloud2 prior_pcd_msg;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> global_map_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> registered_scan_;
    std::vector<sensor_msgs::Imu> imu_buffer_;

    // small gicp
    int num_threads_;
    int num_neighbors_;
    float global_leaf_size_;
    float registered_leaf_size_;
    float max_dist_sq_;
    int max_iterations_;
    int align_time_thre_;

    std::shared_ptr<pcl::PointCloud<pcl::PointCovariance>> target_;
    std::shared_ptr<pcl::PointCloud<pcl::PointCovariance>> source_;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
    std::unique_ptr<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>> register_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // gpu bbs
    bool if_bbs_localize_ = true;
    std::unique_ptr<gpu::BBS3D> gpu_bbs3d_;
    
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> filtered_map_cloud_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> filtered_scan_cloud_;
    
    double current_yaw_;

    Eigen::Vector3d min_rpy_;
    Eigen::Vector3d max_rpy_;
    double min_level_res_;
    int max_level_;
    double score_percentage_thre_;

    float map_leaf_size_;
    float source_leaf_size_;
    
    double min_scan_range_;
    double max_scan_range_;
    int timeout_msec_;
 
    std::vector<Eigen::Vector3f> map_points;
    std::vector<Eigen::Vector3f> source_points;

    // mutex
    std::mutex cloud_mutex_;
    std::mutex gpu_bbs3d_mutex_;
};

const std::string RESET = "\033[0m\n";
const std::string BLACK = "\033[30m";
const std::string RED = "\033[31m";
const std::string GREEN = "\033[32m";
const std::string YELLOW = "\033[33m";
const std::string BLUE = "\033[34m";
const std::string PURPLE_RED = "\033[35m";
const std::string WHITE = "\033[37m"; 
#pragma once

// common
#include <string>
#include <memory>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>

//msg
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//tf
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

//small gicp
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>

class SmallGicpRelocalization: ros::NodeHandle
{
public: 
    SmallGicpRelocalization();
    ~SmallGicpRelocalization();

private:
    void registerPcdCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void loadGlobalMap(const std::string& file_name);

    void setInitialTransformation();

    void runRegistration(const ros::WallTimerEvent& event);

    void publishTransform(const ros::WallTimerEvent& event);

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle pr_nh_;
    ros::Subscriber sub_pcd_;
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
    std::string initialpose_topic_;

    ros::Time last_scan_time_;
    Eigen::Isometry3d initial_transform_;
    Eigen::Isometry3d result_t_;
    Eigen::Isometry3d prior_result_t_;

    ros::WallTimer register_timer_;  
    ros::WallTimer transform_timer_;

    sensor_msgs::PointCloud2 prior_pcd_msg;

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> global_map_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> registered_scan_;


    // small gicp
    int num_threads_;
    int num_neighbors_;
    float global_leaf_size_;
    float registered_leaf_size_;
    float max_dist_sq_;
    int max_iterations_;

    std::shared_ptr<pcl::PointCloud<pcl::PointCovariance>> target_;
    std::shared_ptr<pcl::PointCloud<pcl::PointCovariance>> source_;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
    std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>> register_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::mutex cloud_mutex_;
};

const std::string RESET = "\033[0m";
const std::string BLACK = "\033[30m";
const std::string RED = "\033[31m";
const std::string GREEN = "\033[32m";
const std::string YELLOW = "\033[33m";
const std::string BLUE = "\033[34m";
const std::string PURPLE_RED = "\033[35m";
const std::string WHITE = "\033[37m"; 
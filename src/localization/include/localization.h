#include <string>
#include <memory>
#include <vector>
#include <iostream>

//ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//tf2
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

//third party
#include <pclomp/ndt_omp.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sentry_userdefinition/LocalizationInfo.h>
namespace rm_sentry{
class sentry_localization{
    using PointT = pcl::PointXYZ;
public:
    sentry_localization();
    ~sentry_localization();

    const std::string RESET = "\033[0m";
    const std::string BLACK = "\033[30m";
    const std::string RED = "\033[31m";
    const std::string GREEN = "\033[32m";
    const std::string YELLOW = "\033[33m";
    const std::string BLUE = "\033[34m";
    const std::string PURPLE_RED = "\033[35m";
    const std::string WHITE = "\033[37m"; 


private:

    ros::NodeHandle nh;
    ros::NodeHandle pr_nh;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_livox_mid360;
    ros::Subscriber sub_lidar_points;

    ros::Publisher pub_global_map;
    ros::Publisher pub_lidar_points; 
    ros::Publisher pub_localization_status;
    ros::Publisher pub_gicp_pointcloud;
    ros::Publisher pub_rotate_cloud;
    ros::Publisher pub_removed_pointcloud;
    ros::Publisher pub_localization;
    ros::Publisher pub_relocate_tranform_visuial;
    ros::Publisher pub_relocalization_initialpose;

    ros::ServiceServer relocalization_srv;

    //定位结果
    geometry_msgs::PoseWithCovarianceStamped localization_match;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;


    tf2::Transform base_to_laser_;    
    tf2::Transform laser_to_base_; 

    tf2::Transform base_in_odom_;           // base_link在odom坐标系下的坐标
    tf2::Transform base_in_odom_keyframe_;  // base_link在odom坐标系下的keyframe的坐标
 
    Eigen::Isometry3d base_to_lidar_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d map_to_base_ = Eigen::Isometry3d::Identity();     //map到base的欧式变换矩阵4x4
    Eigen::Isometry3d map_to_lidar_ = Eigen::Isometry3d::Identity();     //map到laser的欧式变换矩阵4x4

    Eigen::Isometry3d match_result_ = Eigen::Isometry3d::Identity();     //icp匹配结果
    Eigen::Isometry3d last_match_result_ = Eigen::Isometry3d::Identity();     //上一帧icp匹配结果

    bool if_debug_;
    bool map_initialized_ = false;
    bool scan_initialized_ = false;
    bool odom_initialized_ = false;
    bool need_relocalization_ = false;
    bool relocalization_result_ = false;

    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string lidar_frame_;


    //lidarcallback用时和频率
    std::chrono::steady_clock::time_point time_start_;
    std::chrono::steady_clock::time_point time_end_;
    std::chrono::steady_clock::time_point time_lastend_;
    std::chrono::duration<double> time_duration_;

    double last_match_time_ = 0;
    double match_time_;
    double scan_time_;

    pcl::PointCloud<PointT>::Ptr cloud_scan;
    pcl::PointCloud<PointT>::Ptr cloud_map;

    //global pcd-map
    std::string pcd_map;
    double downsample_resolution_globalmap;
    pcl::PointCloud<PointT>::Ptr pcdmap_cloud;
    sensor_msgs::PointCloud2 pcdmap_cloud_ros;


    //localization
    int LOCALIZATION_ITERATIONS_THRE;
    double AGE_THRE;
    double DIST_THRE;
    double ANGLE_THRE;
    double ANGLE_UPPER_THRE;
    double ANGLE_SPEED_THRE;
    int POINT_NUM_THRE;
    double SCORE_THRE;

    double Variance_X;
    double Variance_Y;
    double Variance_Yaw;

    double VoxelRemoval_Resolution;
    double OBSTACLE_DISTANCE_THRE;

    //relocalization
    bool need_relocalization;
    
    int LOSS_NUM_THRE;
    int localization_loss_num;
    int RELOCALIZATION_ITERATIONS_THRE;
    float RELOCALIZATION_SCORE_THRE;

    float Relocation_Weight_Score_;
    float Relocation_Weight_Distance_;
    float Relocation_Weight_Yaw_;

    //odom
    std::mutex odom_lock;
    std::deque<nav_msgs::Odometry> odom_queue;
    int odom_queue_length;
    bool odom_initialized;

    std::vector<livox_ros_driver2::CustomMsg::ConstPtr> livox_data;



    void InitParams();
    
    
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odometry_msg);
    
    void LidarCallback(const sensor_msgs::PointCloud2::ConstPtr &lidar_msgs);

    bool ScanMatchWithGICP(Eigen::Isometry3d &trans, pcl::PointCloud<PointT>::Ptr &cloud_scan_msg, pcl::PointCloud<PointT>::Ptr &cloud_map_msg);

    bool RelocalizationWithGICP(Eigen::Isometry3d &trans ,pcl::PointCloud<PointT>::Ptr &cloud_scan_msg, pcl::PointCloud<PointT>::Ptr &cloud_map_msg, const Eigen::Isometry3d &robot_pose);

    void RotatePointCloud(pcl::PointCloud<PointT>::Ptr &cloud_msg, const Eigen::Affine3f &rotation, const Eigen::Affine3f &robot_pose);

    bool GetOdomTransform(Eigen::Isometry3d &trans, double start_time, double end_time);

    bool Get2TimeTransform(Eigen::Isometry3d &trans);
    
    bool GetTransform(Eigen::Isometry3d &trans , const std::string parent_frame, const std::string child_frame, const ros::Time stamp);

    void PointCloudObstacleRemoval(pcl::PointCloud<PointT>::Ptr &cloud_map_msg, pcl::PointCloud<PointT>::Ptr &cloud_msg, double DISTANCE_THRESHOLD);

    void PointCloudOutlierRemoval(pcl::PointCloud<PointT>::Ptr &cloud_msg);

    void PointCloudVoxelGridRemoval(pcl::PointCloud<PointT>::Ptr &cloud_msg, double resolution);

    geometry_msgs::PoseWithCovarianceStamped Isometry3d_to_PoseWithCovarianceStamped(const Eigen::Isometry3d& iso);

    void CustomMsgToPointcloud(const livox_ros_driver2::CustomMsg::ConstPtr &livox_msg_in);

    void PcdmapToROS();

};

}

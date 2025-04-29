#pragma once

// common
#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>

// msgs
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class TerrainAnalysisNode
{
public:
  TerrainAnalysisNode();
  ~TerrainAnalysisNode();
  
private:
    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom);
    void laserCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &laserCloud2);
    void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy);
    void clearingHandler(const std_msgs::Float32::ConstPtr &dis);
    void processLaserCloud();
  
    double scan_voxel_size_;
    double decay_time_;
    double no_decay_dis_;
    double clearing_dis_;
    bool clearing_cloud_;
    bool use_sorting_;
    double quantile_z_;
    bool consider_drop_;
    bool limit_ground_lift_;
    double max_ground_lift_;
    bool clear_dy_obs_;
    double min_dy_obs_dis_;
    double min_dy_obs_angle_;
    double min_dy_obs_rel_z_;
    double abs_dy_obs_rel_z_thre_;
    double min_dy_obs_vfov_;
    double max_dy_obs_vfov_;
    int min_dy_obs_point_num_;
    bool no_data_obstacle_;
    int no_data_block_skip_num_;
    int min_block_point_num_;
    double vehicle_height_;
    int voxel_point_update_thre_;
    double voxel_time_update_thre_;
    double min_rel_z_;
    double max_rel_z_;
    double dis_ratio_z_;
  
    // terrain voxel parameters
    int terrain_voxel_shift_x_ = 0;
    int terrain_voxel_shift_y_ = 0;
    const float terrain_voxel_size_ = 1.0;
    const int terrain_voxel_width_ = 21;
    const int terrain_voxel_half_width_ = (terrain_voxel_width_ - 1) / 2;
    const int terrain_voxel_num_ = terrain_voxel_width_ * terrain_voxel_width_;
  
    // planar voxel parameters
    const float planar_voxel_size_ = 0.2;
    const int planar_voxel_width_ = 51;
    const int planar_voxel_half_width_ = (planar_voxel_width_ - 1) / 2;
    const int planar_voxel_num_ = planar_voxel_width_ * planar_voxel_width_;
  
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_crop_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_dwz_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_elev_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_voxel_cloud_[441];
  
    int terrain_voxel_update_num_[441];
    float terrain_voxel_update_time_[441];
    float planar_voxel_elev_[2601];
    int planar_voxel_edge_[2601];
    int planar_voxel_dy_obs_[2601];
    std::vector<float> planar_point_elev_[2601];
  
    double laser_cloud_time_;
  
    double system_init_time_;
    bool system_inited_;
    int no_data_inited_;
  
    float vehicle_x_rec_, vehicle_y_rec_;
  
    float sin_vehicle_roll_, cos_vehicle_roll_;
    float sin_vehicle_pitch_, cos_vehicle_pitch_;
    float sin_vehicle_yaw_, cos_vehicle_yaw_;
  
    nav_msgs::Odometry odom_;
    pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_;
    
    
    ros::NodeHandle nh_;
    ros::NodeHandle pr_nh_;

    ros::Subscriber sub_odometry_;
    ros::Subscriber sub_laser_cloud_;
    ros::Subscriber sub_joystack_;
    ros::Subscriber sub_clearing_;
    ros::Publisher  pub_terrain_map_;

    std::string odom_topic_;
    std::string cloud_topic_;

};


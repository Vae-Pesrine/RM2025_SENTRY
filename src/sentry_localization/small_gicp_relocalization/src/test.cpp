#include "global_localization/global_localization.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
double max_range_, min_tx_, max_tx_, min_ty_, max_ty_, min_theta_, max_theta_;
double map_min_z_, map_max_z_, map_resolution_;
int map_width_, map_height_;
int max_points_pre_cell_, map_pyramid_level_;
double scan_min_z_, scan_max_z_;
double map_filter_resolution_, scan_filter_resolution_;
double global_map_width_, global_map_height_;
pcl::PointCloud<pcl::PointXYZI>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZI>);

// 点云回调，将ROS点云消息转换为PCL点云
void cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    pcl::fromROSMsg(*cloud_msg, *scan_cloud);
    ROS_INFO("Received scan cloud with %zu points.", scan_cloud->size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_localization_example");
    ros::NodeHandle nh;
    nh.param<double>("bbs/max_range", max_range_, 30.0);
    nh.param<double>("bbs/min_tx", min_tx_, -20.0);
    nh.param<double>("bbs/max_tx", max_tx_, 20.0);
    nh.param<double>("bbs/min_ty", min_ty_, -20.0);
    nh.param<double>("bbs/max_ty", max_ty_, 20.0);
    nh.param<double>("bbs/min_theta", min_theta_, -3.14);
    nh.param<double>("bbs/max_theta", max_theta_, 3.14);
    nh.param<double>("bbs/map_min_z", map_min_z_, -5.0);
    nh.param<double>("bbs/map_max_z", map_max_z_, 15.0);
    nh.param<int>("bbs/map_width", map_width_, 330);
    nh.param<int>("bbs/map_height", map_height_, 283);
    nh.param<double>("bbs/map_resolution", map_resolution_, 0.05);
    nh.param<int>("bbs/max_points_pre_cell", max_points_pre_cell_, 5);
    nh.param<int>("bbs/map_pyramid_level", map_pyramid_level_, 6);
    nh.param<double>("bbs/scan_min_z", scan_min_z_, -1.5);
    nh.param<double>("bbs/scan_max_z", scan_max_z_, 1.5);
    nh.param<double>("bbs/map_filter_resolution", map_filter_resolution_, 0.05);
    nh.param<double>("bbs/scan_filter_resolution", scan_filter_resolution_, 0.05);
    nh.param<double>("global_map_width", global_map_width_, 330.0);
    nh.param<double>("global_map_height", global_map_height_, 283.0);

    // 1. 加载全局地图
    std::string path = ros::package::getPath("small_gicp_relocalization");
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(path + "/PCD/" + "rmuc_2025.pcd", *map_cloud) == -1) {
        PCL_ERROR("Couldn't read global map file\n");
        return -1;
    }
    ROS_INFO("Loaded global map with %zu points.", map_cloud->size());

    // 可选：对地图进行体素滤波
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setLeafSize(0.25f, 0.25f, 0.25f);
    vg.setInputCloud(map_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    vg.filter(*map_cloud_filtered);
    ROS_INFO("Filtered global map to %zu points.", map_cloud_filtered->size());

    // 2. 订阅待定位点云帧
    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 10, cb);

    // 3. 创建全局定位对象并设置地图
    global_localization::GlobalLocalizationBBS global_localizer(nh);
    
    // 构造变换矩阵
    double x = 8.50396;
    double y = 4.052823;
    double z = 0.140688;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 1.570796327; // 弧度

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << x, y, z;
    transform.rotate(Eigen::AngleAxisf(roll,  Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(yaw,   Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*map_cloud_filtered, *map_cloud_transformed, transform);

    // 后续用 map_cloud_transformed 作为全局地图
    global_localizer.set_global_map(map_cloud_transformed);

    // 4. 等待点云帧到来并执行全局定位
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        if (scan_cloud->size() > 0) {
            Eigen::Matrix4f estimated_pose;
            bool success = global_localizer.globalLocalization(scan_cloud, estimated_pose);

            if (success) {
                std::cout << "Global localization succeeded!" << std::endl;
                std::cout << "Estimated pose:\n" << estimated_pose << std::endl;
            } else {
                std::cout << "Global localization failed." << std::endl;
            }
            scan_cloud->clear(); // 清空，等待下一帧
        }
        rate.sleep();
    }

    return 0;
}
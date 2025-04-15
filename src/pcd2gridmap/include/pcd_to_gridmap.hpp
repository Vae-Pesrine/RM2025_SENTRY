#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;


class PCDToGridMap
{
public:
    PCDToGridMap();
    ~PCDToGridMap();

private:

    void pcdFilter();
    void loadPCD();

    void publishCb();

    void buildMap(const PointCloudT::Ptr &cloud, nav_msgs::OccupancyGrid &map);

    // filter
    double thre_z_min_;
    double thre_z_max_;

    double thre_radius_;
    int thre_count_;

    // transform
    double x_, y_, z_;
    double roll_, pitch_, yaw_;
    Eigen::Affine3d transform_;

    double map_resolution_;

    std::string pcd_file_;

    PointCloudT::Ptr pcd_cloud_;
    PointCloudT::Ptr filtered_passcloud_;
    PointCloudT::Ptr filtered_radiuscloud_;

    nav_msgs::OccupancyGrid map_;

    ros::NodeHandle nh_;
    ros::NodeHandle pr_nh_;

    ros::Publisher pub_map_;
    ros::Publisher pub_pcd_;
    ros::Timer timer_;
};





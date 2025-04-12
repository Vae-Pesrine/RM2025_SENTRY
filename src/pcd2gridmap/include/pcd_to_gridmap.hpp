#pragma once

#include <ros/ros.h>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

class PclProcess
{
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

    PclProcess();
    ~PclProcess();

private:
    bool loadPCD(PointCloud::Ptr &cloud_output);
    void pcdLoadCb(const ros::TimerEvent&);



private:
    ros::NodeHandle pr_nh_;
    ros::NodeHandle nh_;
    ros::Publisher pub_pcd_;

    ros::Timer update_timer_;

    std::string pcd_file_;

    double x_, y_, z_;
    double roll_, pitch_, yaw_;

    Eigen::Affine3d transform;
};

























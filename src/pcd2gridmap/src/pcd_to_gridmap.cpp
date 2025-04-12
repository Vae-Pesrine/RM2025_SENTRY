#include "pcd_to_gridmap.hpp"


PclProcess::PclProcess()
    : nh_(),
      pr_nh_("~"),
      transform(Eigen::Affine3d::Identity())
{
    pr_nh_.param<std::string>("pcd_file", pcd_file_, "");
    pr_nh_.param<double>("x", x_, 0.0);
    pr_nh_.param<double>("y", y_, 0.0);
    pr_nh_.param<double>("z", z_, 0.0);
    pr_nh_.param<double>("roll", roll_, 0.0);
    pr_nh_.param<double>("pitch", pitch_, 0.0);
    pr_nh_.param<double>("yaw", yaw_, 0.0);

    // degree to rad
    roll_ = roll_ * M_PI / 180.0;
    pitch_ = pitch_ * M_PI / 180.0;
    yaw_ = yaw_ * M_PI / 180.0;
    
    transform.translation() = Eigen::Vector3d(x_, y_, z_);
    Eigen::AngleAxisd roll_rotation(roll_, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_rotation(pitch_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_rotation(yaw_, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw_rotation * pitch_rotation * roll_rotation;
    transform.linear() = q.toRotationMatrix();

    pub_pcd_ = nh_.advertise<sensor_msgs::PointCloud2>("/pcd", 1, false);

    // 使用 boost::bind 绑定成员函数
    update_timer_ = nh_.createTimer(ros::Duration(1.0), boost::bind(&PclProcess::pcdLoadCb, this, _1));
}

PclProcess::~PclProcess()
{

}

bool PclProcess::loadPCD(PointCloud::Ptr &cloud_output)
{
    auto cloud_load = std::make_shared<PointCloud>();

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *cloud_load) == -1) {
        ROS_ERROR_STREAM("Failed to load PCD file: " << pcd_file_);
        return false; 
    }

    auto cloud_transformed = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*cloud_load, *cloud_transformed, transform);
    *cloud_output = *cloud_transformed;

    return true; 
}

void PclProcess::pcdLoadCb(const ros::TimerEvent&)
{
    ROS_INFO_STREAM("The pcd file is loaded");

    auto cloud_transformed = PointCloud::Ptr(new PointCloud);
    if (!loadPCD(cloud_transformed)) {
        ROS_ERROR_STREAM("Failed to load and transform PCD file.");
        return;
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_transformed, cloud_msg);
    cloud_msg.header.frame_id = "world";

    pub_pcd_.publish(cloud_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_2_gridmap");

    PclProcess pcl_process;

    ros::spin();
}



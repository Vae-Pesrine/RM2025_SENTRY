#include "ros/ros.h"
#include "livox_repub/lidar_data.h"

#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <iostream>
#include <Eigen/Dense>

ros::Subscriber sub_livox;
ros::Publisher pub_livox;

pcl::PointCloud<PointXYZIRT>::Ptr raw_cloud_;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZ>};
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final {new pcl::PointCloud<pcl::PointXYZ>};


void livox_in_callback(livox_ros_driver2::CustomMsgConstPtr lidar_msg)
{
    raw_cloud_->clear();
    cloud_in->clear();
    cloud_final->clear();
    for(const auto& p : lidar_msg->points){
        RTPoint point;
        int line_num = (int)p.line;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.intensity = p.reflectivity;
        point.time = p.offset_time/1e9;
        point.ring = (line_num);

        Eigen::Vector3d epoint(p.x,p.y,p.z);
        raw_cloud_->push_back(point);
    }
    sensor_msgs::PointCloud2 pcd_msg;
    pcl::toROSMsg(*raw_cloud_,pcd_msg);
    pcd_msg.header.stamp = lidar_msg->header.stamp;
    pcd_msg.header.frame_id = "livox_mid360";

    pub_livox.publish(pcd_msg);  
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "livox_repub_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_pr("~");
    ros::Rate loop_rate(100);

    std::string livox_in_name;
    nh_pr.param<std::string>("livox_in_name", livox_in_name, "/livox/lidar");
    std::string livox_out_name;
    nh_pr.param<std::string>("livox_out_name", livox_out_name, "/livox/points");

    raw_cloud_ = std::make_shared<pcl::PointCloud<PointXYZIRT>>();
    
    pub_livox = nh.advertise<sensor_msgs::PointCloud2>(livox_out_name.c_str(), 10);
    sub_livox = nh.subscribe(livox_in_name.c_str(), 1000, livox_in_callback);

    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 1;
}
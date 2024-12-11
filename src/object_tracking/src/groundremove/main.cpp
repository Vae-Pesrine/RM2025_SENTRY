#include <ros/ros.h>

//msg
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>

#include <vector>
#include <iostream>
#include <cmath>
#include <utility>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <gaus_blur.h>

std::string cloud_topic;

ros::Publisher pub_ground;
ros::Publisher pub_nonground;
ros::Publisher pub_auxpoint;

int count = 0;
float filter_z_min;
float filter_z_max;
pcl::ConditionalRemoval<pcl::PointXYZ> condrem;

void filter_mid_area_limitation()
{
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cloud(new pcl::ConditionAnd<pcl::PointXYZ>);
    
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_1(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -15));
    range_cloud->addComparison(cond_1);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_2(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 5));
    range_cloud->addComparison(cond_2);

    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_3(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -50));
    range_cloud->addComparison(cond_3);
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_4(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 50));
    range_cloud->addComparison(cond_4);

    condrem.setCondition(range_cloud);
}
void filter_mid_area(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    condrem.setInputCloud(cloud_in);
    condrem.setKeepOrganized(false);
    condrem.filter(*cloud_in);
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonground_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromROSMsg(*input, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr z_filter_cloud(new pcl::PointCloud<pcl::PointXYZ>()) ;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(filter_z_min, filter_z_max);
    pass.filter(*z_filter_cloud);

    filter_mid_area(z_filter_cloud);

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*z_filter_cloud, ros_cloud);
    ros_cloud.header = input->header;
    pub_auxpoint.publish(ros_cloud);

    groundRemove(z_filter_cloud, nonground_cloud, ground_cloud);

    nonground_cloud->header.frame_id = input->header.frame_id;
    ground_cloud->header.frame_id = input->header.frame_id;
    sensor_msgs::PointCloud2 output_ground;  
    sensor_msgs::PointCloud2 output_nonground;
    pcl::toROSMsg(*ground_cloud, output_ground);
    pcl::toROSMsg(*nonground_cloud, output_nonground);

    pub_nonground.publish(output_nonground);
    pub_ground.publish(output_ground);

    count++;
    std::cout << "ground frame: " << count << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground");
    ros::NodeHandle nh;
    ros::NodeHandle nh_pr("~");

    nh_pr.param<std::string>("cloud_topic", cloud_topic, "/velodyne_points");
    nh_pr.param<float>("filter_z_min", filter_z_min, -1.0);
    nh_pr.param<float>("filter_z_max", filter_z_max, 1.0);
    ros::Subscriber sud_cloud = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic.c_str(), 160, cloud_cb);
    pub_auxpoint = nh.advertise<sensor_msgs::PointCloud2>("aux_points", 1);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("ground_points", 1);
    pub_nonground = nh.advertise<sensor_msgs::PointCloud2>("nonground_points", 1);

    filter_mid_area_limitation();

    ros::spin();
    return 0;

}

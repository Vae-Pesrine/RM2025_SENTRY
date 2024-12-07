#include <cmath>
#include <ros/ros.h>
#include <ros/param.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <tf2/utils.h>

#include "std_cout.h"

geometry_msgs::PoseWithCovarianceStamped initialPose;
double covariance[36];
double pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "initialpose_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_pr("~");
    ros::Publisher pub_initialPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 5);
    ros::Rate loop_rate(50);
    
    nh_pr.param<double>("initialpose/position_x", pos_x, 0.0);
    nh_pr.param<double>("initialpose/position_y", pos_y, 0.0);
    nh_pr.param<double>("initialpose/position_z", pos_z, 0.0);
    nh_pr.param<double>("initialpose/orientation_x", ori_x, 0.0);
    nh_pr.param<double>("initialpose/orientation_y", ori_y, 0.0);
    nh_pr.param<double>("initialpose/orientation_z", ori_z, 0.0);
    nh_pr.param<double>("initialpose/orientation_w", ori_w, 0.0);

    std::cout << BLUE << "--------------------------------" << std::endl
              << GREEN << "pos_x: " << pos_x << std::endl
                       << "pos_y: " << pos_y << std::endl
                       << "pos_z: " << pos_z << std::endl
                       << "ori_x: " << ori_x << std::endl
                       << "ori_y: " << ori_y << std::endl
                       << "ori_z: " << ori_z << std::endl
                       << "ori_w: " << ori_w << std::endl
              << BLUE << "--------------------------------" << RESET << std::endl;


    initialPose.pose.pose.position.x = pos_x;
    initialPose.pose.pose.position.y = pos_y;
    initialPose.pose.pose.position.z = pos_z;

    initialPose.pose.pose.orientation.x = ori_x;
    initialPose.pose.pose.orientation.y = ori_y;
    initialPose.pose.pose.orientation.z = ori_z;
    initialPose.pose.pose.orientation.w = ori_w;


    for(auto i = 0; i < 36; ++i)
    {
        initialPose.pose.covariance[i] = covariance[i];
    }

    int count = 0;
    while(ros::ok())
    {
        initialPose.header.frame_id = "map";
        initialPose.header.stamp = ros::Time();

        ros::spinOnce();
        count++;
        if(count <= 250)
        {
            pub_initialPose.publish(initialPose);
            // ROS_INFO_STREAM("The pose has been published!");            
        }
        loop_rate.sleep();
    }
    
    return 0;
}

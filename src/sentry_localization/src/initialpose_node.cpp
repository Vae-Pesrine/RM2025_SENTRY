#include <cmath>
#include <array>
#include <iostream>
#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_cout.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "initialpose_node");
    ros::NodeHandle nh;
    ros::NodeHandle pr_nh("~");
    auto pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 5);
    ros::Rate loop_rate(10);
    int pub_time;

    std::array<double, 36> covariance{};
    double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
    double ori_x = 0.0, ori_y = 0.0, ori_z = 0.0, ori_w = 0.0;

    pr_nh.param<double>("position_x", pos_x, 0.0);
    pr_nh.param<double>("position_y", pos_y, 0.0);
    pr_nh.param<double>("position_z", pos_z, 0.0);
    pr_nh.param<double>("orientation_x", ori_x, 0.0);
    pr_nh.param<double>("orientation_y", ori_y, 0.0);
    pr_nh.param<double>("orientation_z", ori_z, 0.0);
    pr_nh.param<double>("orientation_w", ori_w, 0.0);
    pr_nh.param<int>("pub_time", pub_time, 4);

    // 打印初始位姿信息
    std::cout << "--------------------------------" << std::endl
              << "pos_x: " << pos_x << std::endl
              << "pos_y: " << pos_y << std::endl
              << "pos_z: " << pos_z << std::endl
              << "ori_x: " << ori_x << std::endl
              << "ori_y: " << ori_y << std::endl
              << "ori_z: " << ori_z << std::endl
              << "ori_w: " << ori_w << std::endl
              << "--------------------------------" << std::endl;

    geometry_msgs::PoseWithCovarianceStamped initialPose;
    initialPose.header.frame_id = "map";
    initialPose.pose.pose.position.x = pos_x;
    initialPose.pose.pose.position.y = pos_y;
    initialPose.pose.pose.position.z = pos_z;
    initialPose.pose.pose.orientation.x = ori_x;
    initialPose.pose.pose.orientation.y = ori_y;
    initialPose.pose.pose.orientation.z = ori_z;
    initialPose.pose.pose.orientation.w = ori_w;

    for (size_t i = 0; i < covariance.size(); ++i) {
        initialPose.pose.covariance[i] = covariance[i];
    }

    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        if ((current_time - start_time).toSec() > pub_time) {
            break; 
        }

        initialPose.header.stamp = ros::Time::now();
        pub_pose.publish(initialPose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << GREEN << "Initial pose publishing stopped after " << pub_time << " s" 
              << RESET << std::endl;
    return 0;
}
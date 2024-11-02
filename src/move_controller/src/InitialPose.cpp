#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::PoseWithCovarianceStamped InitialPose;
geometry_msgs::Quaternion geoquat;
double roll, pitch, yaw;
double covariance[36];
double pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "InitialPose");
    ros::NodeHandle nh;
    ros::NodeHandle nh_pr("~");
    ros::Publisher pub_initialPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_ori", 5);
    ros::Rate rate(50);
    
    nh_pr.param<double>("position_x", pos_x, 0.0);
    nh_pr.param<double>("position_y", pos_y, 0.0);
    nh_pr.param<double>("position_z", pos_z, 0.0);
    nh_pr.param<double>("orientation_x", ori_x, 0.0);
    nh_pr.param<double>("orientation_y", ori_y, 0.0);
    nh_pr.param<double>("orientation_z", ori_z, 0.0);
    nh_pr.param<double>("orientation_w", ori_w, 0.0);

    ROS_INFO_STREAM(pos_x);

    InitialPose.pose.pose.position.x = pos_x;
    InitialPose.pose.pose.position.y = pos_y;
    InitialPose.pose.pose.position.z = pos_z;

    InitialPose.pose.pose.orientation.x = ori_x;
    InitialPose.pose.pose.orientation.y = ori_y;
    InitialPose.pose.pose.orientation.z = ori_z;
    InitialPose.pose.pose.orientation.w = ori_w;

    geoquat=tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    InitialPose.pose.pose.orientation = geoquat;
    for(int i = 0; i < 36; i++)
    {
     InitialPose.pose.covariance[i] = covariance[i];
    }

    int count = 0;
    while(ros::ok())
    {
        InitialPose.header.frame_id = "map";
        InitialPose.header.stamp = ros::Time();
        ros::spinOnce();
        rate.sleep();
        count++;
        if(count <= 250)
            pub_initialPose.publish(InitialPose);
    }
    
    return 0;
}

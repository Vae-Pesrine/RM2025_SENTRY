#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;

geometry_msgs::PoseWithCovarianceStamped InitialPose;
geometry_msgs::Quaternion geoquat;
double roll, pitch, yaw;
double covariance[36];

int main(int argc, char *argv[])
{
    /* code */
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "InitialPose");
    ros::NodeHandle nh;
    ros::Publisher pub_InitialPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_ori",5);
    // ros::Subscriber sub_Odom = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 10, odometryCallback);
    // ros::Subscriber sub_refree = nh.subscribe<usr_fun::refree>("refree",100,refreeCallback);
    ros::Rate rate(50);
    
    InitialPose.header.frame_id="map";
    
    geoquat=tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    InitialPose.pose.pose.orientation = geoquat;
    for(int i=0;i<36;i++)
    {
     InitialPose.pose.covariance[i]=covariance[i];
    }
    
    // 2024 RMUC
    InitialPose.pose.pose.position.z = 0.0;
    InitialPose.pose.pose.position.y = 2.36;
    InitialPose.pose.pose.position.x = 0.08332;

    InitialPose.pose.pose.orientation.z = 0.86378;
    InitialPose.pose.pose.orientation.y = 0.0;
    InitialPose.pose.pose.orientation.x = 0.0;
    InitialPose.pose.pose.orientation.w = 0.50386;

    
    bool flag = true;
    int count = 0;
    while(ros::ok())
    {
        InitialPose.header.stamp = ros::Time();
        ros::spinOnce();
        rate.sleep();
        count++;
        if(count <= 250)
        pub_InitialPose.publish(InitialPose);
    }
    
    return 0;
}

// common
#include <iostream>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <livox_msgs_convert/livox_converter.hpp>

class LivoxToPointcloud2
{
public:
    LivoxToPointcloud2():
        nh("~"){
        points_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/points", 10);
        points_sub = nh.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar", 10, &LivoxToPointcloud2::livoxHandler, this);
    }

    void livoxHandler(
        const livox_ros_driver2::CustomMsg::ConstPtr &livox_msg
    ) {
        const auto points_msg = converter.convert(*livox_msg);
        points_pub.publish(points_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber points_sub;
    ros::Publisher points_pub;

    LivoxConverter converter;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_to_pointcloud2");

    LivoxToPointcloud2 convert_node;
    ros::spin();

    return 0;
}
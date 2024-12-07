#include <cmath>
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <tf2/utils.h>

#include "std_cout.h"

geometry_msgs::PoseWithCovarianceStamped initialPose;
geometry_msgs::Quaternion geoquat;
// double roll, pitch, yaw;
double covariance[36];
double pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w;
double dist_range, angle_range, dist_resolution, angle_resolution;

nav_msgs::OccupancyGrid map;
sensor_msgs::LaserScan laser;

std::vector<std::pair<double, double>> cos_sin_table{};
bool got_laser_info = false;

int rangeRelocate(geometry_msgs::PoseWithCovarianceStamped& best_pose,
                  double dist_range, double angle_range, double dist_resolution, double angle_resolution)
{
    auto mapValid = [&](double x, double y) {
        auto i = std::floor((x - map.info.origin.position.x) / map.info.resolution + 0.5);
        auto j = std::floor((y - map.info.origin.position.y) / map.info.resolution + 0.5);
        return (i >= 0 && j >= 0 && i < map.info.width && j < map.info.height);
    };

    auto mapObstacle = [&](double x, double y) {
        auto i = std::floor((x - map.info.origin.position.x) / map.info.resolution + 0.5);
        auto j = std::floor((y - map.info.origin.position.y) / map.info.resolution + 0.5);
        auto idx = i + j * map.info.width;
        return map.data[idx] == 100;
    };

    auto calcuScore = [&](double x, double y, double cos, double sin) {
        const double laser2base = 0.175;
        auto score = 0;
        
        // transform to laser frame to base_link frame
        auto laser_x = x + laser2base * cos;
        auto laser_y = y;
        for (auto i = 0; i < laser.ranges.size(); i += 10) {
            if (laser.ranges[i] < laser.range_min || laser.ranges[i] >= laser.range_max) continue;
            auto cos_pth = cos * cos_sin_table[i].first - sin * cos_sin_table[i].second;
            auto sin_pth = sin * cos_sin_table[i].first + cos * cos_sin_table[i].second;
            auto px = laser_x + laser.ranges[i] * cos_pth;
            auto py = laser_y + laser.ranges[i] * sin_pth;
            if (!mapValid(px, py)) continue;
            if (mapObstacle(px, py)) ++score;
        }
        return score;
    };

    auto min_x = best_pose.pose.pose.position.x - dist_range / 2.0;
    auto max_x = best_pose.pose.pose.position.x + dist_range / 2.0;
    auto min_y = best_pose.pose.pose.position.y - dist_range / 2.0;
    auto max_y = best_pose.pose.pose.position.y + dist_range / 2.0;
    auto min_th = tf2::getYaw(best_pose.pose.pose.orientation) - angle_range / 2.0;
    auto max_th = tf2::getYaw(best_pose.pose.pose.orientation) + angle_range / 2.0;

    int score = 0;
    double target_x, target_y, target_th;
    for (auto th = min_th; th < max_th; th += angle_resolution) {
        auto cos_th = std::cos(th);
        auto sin_th = std::sin(th);
        for (auto x = min_x; x <= max_x; x += dist_resolution) {
          for (auto y = min_y; y <= max_y; y += dist_resolution) {
            if (!mapValid(x, y) || mapObstacle(x, y)) continue;
            auto temp = calcuScore(x, y, cos_th, sin_th);
            if (temp > score) {
              score = temp;
              target_x = x;
              target_y = y;
              target_th = th;
            }
          }
        }
    }

    best_pose.pose.pose.position.x = target_x;
    best_pose.pose.pose.position.y = target_y;
    best_pose.pose.pose.orientation.z = std::sin(target_th / 2.0);
    best_pose.pose.pose.orientation.w = std::cos(target_th / 2.0);
    return score;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapmsg)
{
    map = *mapmsg;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
    laser = *laser_msg;

    if(got_laser_info) return;

    auto start_time = ros::Time::now();
    got_laser_info = true;

    cos_sin_table.resize(laser.ranges.size());
    for(int i = 0; i < laser.ranges.size(); ++i){
        cos_sin_table[i].first = std::cos(laser.angle_min + i * laser.angle_increment);
        cos_sin_table[i].second = std::sin(laser.angle_min + i * laser.angle_increment);
    }
    ROS_WARN_STREAM("Calculate table size" << cos_sin_table.size() << "time used" << (ros::Time::now() - start_time).toSec());

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "InitialPose");
    ros::NodeHandle nh;
    ros::NodeHandle nh_pr("~");
    ros::Subscriber sub_map = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber sub_laser = nh.subscribe("/scan", 1, laserCallback);
    ros::Publisher pub_initialPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 5);
    ros::Rate loop_rate(50);
    
    nh_pr.param<double>("initialpose/position_x", pos_x, 0.0);
    nh_pr.param<double>("initialpose/position_y", pos_y, 0.0);
    nh_pr.param<double>("initialpose/position_z", pos_z, 0.0);
    nh_pr.param<double>("initialpose/orientation_x", ori_x, 0.0);
    nh_pr.param<double>("initialpose/orientation_y", ori_y, 0.0);
    nh_pr.param<double>("initialpose/orientation_z", ori_z, 0.0);
    nh_pr.param<double>("initialpose/orientation_w", ori_w, 0.0);

    nh_pr.param<double>("relocation/dist_range", dist_range, 2.0);
    nh_pr.param<double>("relocation/angle_range", angle_range, 60.0 * 3.14 / 180.0);
    nh_pr.param<double>("relocation/dist_resolution", dist_resolution, 0.2);
    nh_pr.param<double>("relocation/angle_resolution", angle_resolution, 1.0 * 3.14 / 180.0);

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

    // geoquat=tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    // initialPose.pose.pose.orientation = geoquat;

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
            auto start_time = ros::Time::now();
            auto score = rangeRelocate(initialPose, dist_range, angle_range, dist_resolution, angle_resolution);
            ROS_INFO_STREAM("Get new best pose: " << initialPose.pose.pose.position.x << initialPose.pose.pose.position.y << initialPose.pose.pose.position.z <<
                            " score: " << score << " time used" << (ros::Time::now() - start_time).toSec());
            pub_initialPose.publish(initialPose);            

        }
        loop_rate.sleep();
    }
    
    return 0;
}

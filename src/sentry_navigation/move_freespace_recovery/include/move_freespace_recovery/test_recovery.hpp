#pragma once

// ros
#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_layer.h>

// msgs
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// tf
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace move_freespace_recovery
{
class MoveFreeSpace: public nav_core::RecoveryBehavior
{
public:
    MoveFreeSpace();
    ~MoveFreeSpace();

    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap) override;

    void runBehavior() override;

private:
    costmap_2d::Costmap2DROS* global_costmap_;
    costmap_2d::Costmap2DROS* local_costmap_;

    std::string name_;

    tf2_ros::Buffer* tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool initialized_;

    ros::NodeHandle pr_nh_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_cmd_;
    
    ros::Time last_run_time_;

    double twist_x_, twist_y_;
    double max_radius_, robot_radius_;

    int free_thre_;
    bool visualization_;
};
} // namespace move_freespace_recovery
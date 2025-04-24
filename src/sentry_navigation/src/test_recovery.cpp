#include <vector>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>

#include "test_recovery.hpp"

PLUGINLIB_EXPORT_CLASS(move_freespace_recovery::MoveFreeSpace, nav_core::RecoveryBehavior)

namespace  move_freespace_recovery{
MoveFreeSpace::MoveFreeSpace():
    pr_nh_("~"),
    tf_buffer_(nullptr),
    initialized_(false),
    global_costmap_(nullptr),
    local_costmap_(nullptr){
}

MoveFreeSpace::~MoveFreeSpace(){
}

void MoveFreeSpace::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
    if(!initialized_){
        name_ = name;
        tf_buffer_ = tf;
        local_costmap_ = local_costmap;
        global_costmap_ = global_costmap;
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pr_nh_.param<double>("twist_x", twist_x_, 0.0);
        pr_nh_.param<double>("twist_y", twist_y_, 0.0);
        pr_nh_.param<double>("max_radius", max_radius_, 0.0);
        pr_nh_.param<double>("robot_radius", robot_radius_, 0.0);
        pr_nh_.param<int>("free_threshold", free_thre_, 0);
        pr_nh_.param<bool>("visualization", visualization_, false);

        pub_marker_ = pr_nh_.advertise<visualization_msgs::Marker>("/free_markers", 1);
        pub_cmd_ = pr_nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        if (max_radius_ < robot_radius_){
            ROS_WARN("Max radius is smaller than robot radius. Setting max radius to robot radius");
            max_radius_ = robot_radius_;
        }

        ROS_INFO("MoveFreeSpace recovery behavior initialized with name: %s", name_.c_str());

        initialized_ = true;
    }

}

void MoveFreeSpace::runBehavior(){

    if(!initialized_){
        ROS_ERROR("MoveFreeSpace recovery behavior has not been initialized, doing nothing");
        return;
    }


    // 在 runBehavior 开头添加
    ros::Time now = ros::Time::now();
    if ((now - last_run_time_).toSec() < 0.1) { // 限制10Hz
      return;
    }
    last_run_time_ = now;

    ros::Time query_time = now - ros::Duration(0.1);
    geometry_msgs::TransformStamped robot_pose;
    try {
        robot_pose = tf_buffer_->lookupTransform(
            "map", 
            "base_link", 
            query_time, ros::Duration(0.5));
    } catch (tf2::TransformException & ex) {
        ROS_WARN_STREAM("Could not transform: " << ex.what());
        return;
    }

    auto robot_x = robot_pose.transform.translation.x;
    auto robot_y = robot_pose.transform.translation.y;
    auto robot_yaw = tf2::getYaw(robot_pose.transform.rotation);

    bool free_space_found = false;

    auto radius = robot_radius_;
    std::vector<geometry_msgs::Point> free_points;

    auto costmap = global_costmap_->getCostmap();
    auto size_x = costmap->getSizeInCellsX();
    auto size_y = costmap->getSizeInCellsY();
    auto resolution = costmap->getResolution();
    auto origin_x = costmap->getOriginX();
    auto origin_y = costmap->getOriginY();

    // 1. find free space
    while(!free_space_found && radius < max_radius_){
        int free_space_num = 0;
        for(int i = 0; i < size_x; ++i){
            for(int j = 0; j < size_y; ++j){
                double wx = origin_x + (i + 0.5) * resolution;
                double wy = origin_y + (j + 0.5) * resolution;
                double distance_to_center = std::hypot(wx - robot_x, wy - robot_y);
                if(distance_to_center <= radius){
                    unsigned char cost = costmap->getCost(i, j);
                    if(cost == costmap_2d::FREE_SPACE){
                        free_space_num++;
                        geometry_msgs::Point point;
                        point.x = wx;
                        point.y = wy;
                        point.z = 0.0;
                        free_points.emplace_back(point);
                    }
                }
            }
        }

        if(free_space_num > free_thre_){
            free_space_found = true;
            ROS_WARN("Found free space at radius: %f", radius);
            break;
        } else{
            ROS_WARN("No free space found at radius: %f", radius);
            radius += 0.1;
        }
    }

    // 2. compute free space center
    if(free_points.empty()){
        ROS_ERROR("No free space found, aborting recovery behavior");
    }

    auto free_center_x = 0.0;
    auto free_center_y = 0.0;
    for(auto& point: free_points){
        free_center_x += point.x;
        free_center_y += point.y;
    }
    free_center_x /= free_points.size();
    free_center_y /= free_points.size();
    ROS_WARN("Free center: (%f, %f)", free_center_x, free_center_y);

    // 3. visualize free space
    if(visualization_){
        ros::Time current_time = ros::Time::now();
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker marker;
        marker.header.frame_id = global_costmap_->getGlobalFrameID();
        marker.header.stamp = current_time;
        marker.ns = "free_space";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = resolution;
        marker.scale.y = resolution;
        marker.color.r = 1.0;
        marker.color.a = 1.0;
        for(auto& point: free_points){
            marker.points.emplace_back(point);
        }
        markers.markers.emplace_back(marker);

        visualization_msgs::Marker goal_marker;
        goal_marker.header.frame_id = global_costmap_->getGlobalFrameID();
        goal_marker.header.stamp = current_time;
        goal_marker.ns = "goal";
        goal_marker.id = 0;
        goal_marker.type = visualization_msgs::Marker::POINTS;
        goal_marker.action = visualization_msgs::Marker::ADD;
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.scale.x = resolution;
        goal_marker.scale.y = resolution;
        goal_marker.color.g = 1.0;
        goal_marker.color.a = 1.0;
        geometry_msgs::Point goal_point;
        goal_point.x = free_center_x;
        goal_point.y = free_center_y;
        goal_point.z = 0.0;
        goal_marker.points.emplace_back(goal_point);
        markers.markers.emplace_back(goal_marker);
        pub_marker_.publish(markers);
    }

    // 4. compute angle difference
    auto free_center_yaw = atan2(free_center_y - robot_y, free_center_x - robot_x);
    auto angle_diff = free_center_yaw - robot_yaw;
    if(angle_diff > M_PI){
        angle_diff -= 2 * M_PI;
    } else if(angle_diff < -M_PI){
        angle_diff += 2 * M_PI;
    }
    ROS_WARN("Angle diff: %f degree", angle_diff * 180.0 / M_PI);

    // 5. publish cmd_vel
    geometry_msgs::Twist cmd_vel;
    double speed = 0.2;
    cmd_vel.linear.x = speed * cos(angle_diff);
    cmd_vel.linear.y = speed * sin(angle_diff);
    cmd_vel.angular.z = speed * angle_diff;
    pub_cmd_.publish(cmd_vel);
    ROS_WARN("Publishing cmd_vel: (%f, %f, %f)", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

}
} // namespace  move_freespace_recovery
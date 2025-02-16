#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>

// graph-based planner
#include "path_planner/path_planner_node.hpp"
#include "path_planner/graph_planner/astar_planner.hpp"
#include "path_planner/graph_planner/hybrid_astar_planner.hpp"

#include "motion_planning/util/visualizer.hpp"

PLUGINLIB_EXPORT_CLASS(path_planner::PathPlannerNode, nav_core::BaseGlobalPlanner)

namespace path_planner
{
using Visualizer = common::util::Visualizer;

PathPlannerNode::PathPlannerNode(): initialized_(false), global_planner_(nullptr)
{
}


PathPlannerNode::PathPlannerNode(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}

void PathPlannerNode::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos)
{
    costmap_ros_ = costmapRos;
    initialize(name);
}

void PathPlannerNode::initialize(std::string name)
{
    if(!initialized_){
        initialized_ = true;

        ros::NodeHandle pr_nh("~/" + name);

        pr_nh.param<double>("default_tolerance", tolerance_, 0.0);
        pr_nh.param<bool>("outline_map", is_outline_, false);
        pr_nh.param<double>("obstacle_factor", factor_, 0.5);
        pr_nh.param<bool>("expand_zone", is_expand_, false);

        pr_nh.param<std::string>("planner_name", planner_name_, (std::string)"a_star");
        if(planner_name_ == "a_star"){
            global_planner_ = std::make_shared<AStarPathPlanner>(costmap_ros_);
            planner_type_ = GRAPH_PLANNER;
        } else if(planner_name_ == "hybrid_a_star"){
            bool is_reverse;
            double max_curv;
            pr_nh.param<bool>("is_reverse", is_reverse, false);
            pr_nh.param<double>("max_curv", max_curv, 1.0);
            global_planner_ = std::make_shared<HybridAStarPathPlanner>(costmap_ros_, is_reverse, max_curv);
            planner_type_ = GRAPH_PLANNER;
        } else{
            ROS_ERROR_STREAM("Unknown planner name: " << planner_name_);
        }

        global_planner_->setFactor(factor_);

        pub_plan_ = pr_nh.advertise<nav_msgs::Path>("/plan", 1);
        pub_tree_ = pr_nh.advertise<visualization_msgs::MarkerArray>("/random_tree", 1);
        pub_particles_ = pr_nh.advertise<visualization_msgs::MarkerArray>("/particles", 1);

        pub_expand_ = pr_nh.advertise<nav_msgs::OccupancyGrid>("/expand", 1);

        make_plan_srv_ = pr_nh.advertiseService("/make_plan", &PathPlannerNode::makePlanService, this);
    } else{
        ROS_WARN_STREAM("This planner has already been initialized!");
    }
}

bool PathPlannerNode::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan)
{
    return makePlan(start, goal, tolerance_, plan);
}

bool PathPlannerNode::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                               std::vector<geometry_msgs::PoseStamped>& plan)
{
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*global_planner_->getCostMap()->getMutex());
    if(!initialized_){
        ROS_ERROR_STREAM("This planner has not been initialized yet, but it is being used, please call initialize() before use!");
        return false;
    }
    plan.clear();

    if(goal.header.frame_id != frame_id_){
        ROS_ERROR_STREAM("The goal pose passed to this planner must be in the " << frame_id_ << " frame. It is instead in the " << goal.header.frame_id << " frame");
        return false;
    }
    if(start.header.frame_id != frame_id_){
        ROS_ERROR_STREAM("The start pose passed to this planner must be in the " << frame_id_ << " frame. It is instead in the " << start.header.frame_id << " frame");
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;
    double g_start_x, g_start_y, g_goal_x, g_goal_y;
    if(!global_planner_->world2Map(wx, wy, g_start_x, g_start_y)){
        ROS_WARN_STREAM("The robot's start position position is off the global costmap!");
        return false;
    }
    if(!global_planner_->world2Map(wx, wy, g_goal_x, g_goal_y)){
        ROS_WARN_STREAM("The goal sent to the global planner is off the global costmap.");
        return false;
    }

    if(is_outline_){
        global_planner_->outlineMap();
    }

    PathPlanner::Points3d origin_path;
    PathPlanner::Points3d expand;
    bool path_found = false;

    path_found = global_planner_->plan({g_start_x, g_start_y, tf2::getYaw(start.pose.orientation)},
                                      {g_goal_x, g_goal_y, tf2::getYaw(goal.pose.orientation)}, origin_path, expand);
    if(path_found){
        if(_getPlanFromPath(origin_path, plan)){
            geometry_msgs::PoseStamped goalCopy = goal;
            goalCopy.header.stamp = ros::Time::now();
            plan.push_back(goalCopy);
            PathPlanner::Points3d origin_plan, pure_plan;
            for(const auto& pt : plan){
                origin_plan.emplace_back(pt.pose.position.x, pt.pose.position.y);
            }
            const auto& visualizer = common::util::VisualizerPtr::Instance();
            if(is_expand_){
                if(planner_type_ == GRAPH_PLANNER){
                    visualizer->publishExpandZone(expand, costmap_ros_->getCostmap(), pub_expand_, frame_id_);
                } else{
                    ROS_WARN_STREAM("Unknown planner type!");
                }
            }
            visualizer->publishPlan(origin_plan, pub_plan_, frame_id_);
        } else{
            ROS_ERROR_STREAM("Failed to get a plan from when a legal path was found!");
        }
    } else{
        ROS_ERROR_STREAM("Failed to get a path!");
    }
}

bool PathPlannerNode::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
    makePlan(req.start, req.goal, resp.plan.poses);
    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

bool PathPlannerNode::_getPlanFromPath(PathPlanner::Points3d& path, std::vector<geometry_msgs::PoseStamped>& plan)
{
    if(!initialized_){
        ROS_ERROR_STREAM("This planner has not been initialized yet!");
        return false;
    }
    plan.clear();

    for(const auto& pt : path){
        double wx, wy;
        global_planner_->map2World(pt.x(), pt.y(), wx, wy);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    return !plan.empty();
}

} // namespace path_planner
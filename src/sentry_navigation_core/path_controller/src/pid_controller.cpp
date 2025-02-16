#include <pluginlib/class_list_macros.h>
#include "path_controller/pid_controller.hpp"

PLUGINLIB_EXPORT_CLASS(path_controller::PIDController, nav_core::BaseLocalPlanner)

namespace path_controller
{
PIDController::PIDController(): initialized_(false), goal_reached_(false), tf_(nullptr)
{
}

PIDController::PIDController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros): PIDController()
{
    initialize(name, tf, costmap_ros);
}

PIDController::~PIDController()
{
}

void PIDController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_){
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        ros::NodeHandle nh = ros::NodeHandle("~" + name);

        nh.param<double>("goal_dist_tolerance", goal_dist_tolerance_, 0.2);
        nh.param<double>("rotate_tolerance", rotate_tolerance_, 0.5);
        nh.param<std::string>("base_frame", base_frame_, "");
        nh.param<std::string>("map_frame", map_frame_, "");
        nh.param<std::string>("odom_frame", odom_frame_, "");

        // lookahead
        nh.param<double>("lookahead_time", lookahead_time_, 0.5);
        nh.param<double>("min_lookahead_dist", min_lookahead_dist_, 0.3);
        nh.param<double>("max_lookahead_dist", max_lookahead_dist_, 0.9);

        // linear velocity
        nh.param<double>("max_v", max_v_, 0.5);
        nh.param<double>("min_v", min_v_, 0.0);
        nh.param<double>("max_v_inc", max_v_inc_, 0.5);

        // angular velocity
        nh.param<double>("max_w", max_w_, 1.57);
        nh.param<double>("min_w", min_w_, 0.0);
        nh.param<double>("max_w_inc", max_w_inc_, 1.57);

        // PID parameters
        nh.param<double>("k_v_p", k_v_p_, 1.00);
        nh.param<double>("k_v_i", k_v_i_, 0.01);
        nh.param<double>("k_v_d", k_v_d_, 0.10);
        nh.param<double>("k_w_p", k_w_p_, 1.00);
        nh.param<double>("k_w_i", k_w_i_, 0.01);
        nh.param<double>("k_w_d", k_w_d_, 0.10);
        nh.param<double>("k_theta", k_theta_, 0.5);
        nh.param<double>("k", k_, 1.0);
        nh.param<double>("l", l_, 0.2);

        e_v_ = i_v_ = 0.0;
        e_w_ = i_w_ = 0.0;

        nh.param<double>("controller_frequency", controller_frequency_, 10.0);
        d_t_ = 1.0 / controller_frequency_;

        pub_target_pose_  = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
        pub_current_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

        initialized_ = true;
        ROS_INFO_STREAM("PID Controller initialized!");
    } else{
        ROS_WARN_STREAM("PID Controller has already been initialized!");
    }
}

bool PIDController::setPlan(const std::vector<geometry_msgs::PoseStamped>& ori_global_plan)
{
    if(!initialized_){
        ROS_ERROR_STREAM("This local planner has not been initialized!");
        return false;
    } else{
        global_plan_.clear();
        global_plan_ = ori_global_plan;
        if(goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y){
            goal_x_ = global_plan_.back().pose.position.x;
            goal_y_ = global_plan_.back().pose.position.y;
            goal_theta_ = getYawAngle(global_plan_.back());
            goal_reached_ = false;
        }

        return true;
    }
}

bool PIDController::isGoalReached()
{
    if(!initialized_){
        ROS_ERROR_STREAM("This local planner has not been initialized!");
        return false;
    } else{
        if(goal_reached_){
            ROS_INFO_STREAM("Goal Reached!");
            return true;
        }
        return false;
    }
}

bool PIDController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(!initialized_){
        ROS_ERROR_STREAM("This local planner has not been initialized!");
        return false;
    } else{
        nav_msgs::Odometry base_odom;
        odom_helper_->getOdom(base_odom);
        
        // 获取机器人在里程坐标系中的位置, 并将其转换到地图坐标系map中
        geometry_msgs::PoseStamped current_ps_odom, current_ps_map, target_ps_map;
        costmap_ros_->getRobotPose(current_ps_odom);
        transformPose(tf_, map_frame_, current_ps_odom, current_ps_map);
        
        std::vector<geometry_msgs::PoseStamped> remove_plan = removePoints(current_ps_map);

        // 计算前瞻距离
        double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
        double wt = base_odom.twist.twist.angular.z;
        double dist = getLookAheadDistance(vt);

        //获取前瞻点
        geometry_msgs::PointStamped lookahead_pt;
        double theta_d;       // 目标方向
        double theta_dir;     // 从机器人当前位置指向前瞻点的方向
        double theta_trj;     // 路径在前瞻点处的切线方向
        double curvature;     // 路径在前瞻点处的曲率
        getLookAheadPoint(dist, current_ps_map, remove_plan, lookahead_pt, theta_trj, curvature);
    
        target_ps_map.pose.position.x = lookahead_pt.point.x;
        target_ps_map.pose.position.y = lookahead_pt.point.y;
        theta_dir = atan2(target_ps_map.pose.position.y - current_ps_map.pose.position.y, target_ps_map.pose.position.x - current_ps_map.pose.position.x);
        theta_d = regularizeAngle((1 - k_theta_) * theta_trj + k_theta_ * theta_dir);
    
        // 将目标方向theta_d设置为目标姿态的四元数表示
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_d);
        tf2::convert(q, target_ps_map.pose.orientation);

        // 提取机器人当前姿态的偏航角
        double theta = tf2::getYaw(current_ps_map.pose.orientation);

        // 检查是否达到目标位置
        if(shouldRotateToGoal(current_ps_map, global_plan_.back())){
            double e_theta = regularizeAngle(goal_theta_ - theta);
            if(!shouldRotateToPath(std::fabs(e_theta))){
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                goal_reached_ = true;
            } else{
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = angularRegularization(base_odom, e_theta / d_t_);
            }
        } else{
            // 当前状态
            Eigen::Vector3d s(current_ps_map.pose.position.x, current_ps_map.pose.position.y, theta);
            //目标状态
            Eigen::Vector3d s_d(target_ps_map.pose.position.x, target_ps_map.pose.position.y, theta_d);
            // 参考输入
            Eigen::Vector2d u_r(vt, wt);
            Eigen::Vector2d u = _pidControl(s, s_d, u_r);

            cmd_vel.linear.x = linearRegularization(base_odom, u[0]);
            cmd_vel.angular.z = angularRegularization(base_odom, u[1]);
        }

        target_ps_map.header.frame_id = "map";
        target_ps_map.header.stamp = ros::Time::now();
        pub_target_pose_.publish(target_ps_map);

        current_ps_map.header.frame_id = "map";
        current_ps_map.header.stamp = ros::Time::now();            
        pub_current_pose_.publish(current_ps_map);

        return true;
    }
}

Eigen::Vector2d PIDController::_pidControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r)
{
  Eigen::Vector2d u;
  Eigen::Vector3d e(s_d - s);
  Eigen::Vector2d sx_dot(k_ * e[0], k_ * e[1]);
  Eigen::Matrix2d R_inv;
  R_inv << cos(s[2]), sin(s[2]), -sin(s[2]) / l_, cos(s[2]) / l_;
  u = R_inv * sx_dot;

  return u;
}

} // namespace path_controller
#include <tf2/utils.h>

#include "motion_planning/math/math_tools.hpp"
#include "path_controller/controller.hpp"

namespace path_controller
{
PathController::PathController()
    :factor_(0.5), base_frame_("base_link"), map_frame_("map") , odom_frame_("odom"), costmap_ros_(nullptr)
{
}

PathController::~PathController()
{
}

/**
 * @brief 设置或重置障碍物因子
 * @param factor 障碍物因子, 值越大表示障碍物的影响越大
 */
void PathController::setFactor(double factor)
{
    factor_ = factor;
}

/**
 * @brief 设置或重置坐标系名称
 * @param base_frame 机器人基坐标系名称
 * @param map_frame 地图坐标系名称
 */
void PathController::setBaseFrame(std::string base_frame)
{
    base_frame_ = base_frame;
}

void PathController::setmapFrame(std::string map_frame)
{
    map_frame_ = map_frame;
}

/**
 * @brief 将角度标准化到 [-pi, pi]的范围内
 * @param angle 需要标准化的角度
 * @return reg_angle 标准化后的角度
 */
double PathController::regularizeAngle(double angle)
{
    double reg_angle = angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
    return reg_angle;
}

/**
 * @brief 从PoseStamped中获取偏航角
 * @param ps PoseStamped对象, 用于计算n偏航角
 * @return yaw 偏航角
 */
double PathController::getYawAngle(geometry_msgs::PoseStamped& ps)
{
    tf2::Quaternion q(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

/**
 * @brief 判断机器人是否需要通过旋转操作到达目标姿态
 * @param cur_pose 机器人当前姿态
 * @param goal_pose 机器人目标姿态
 * @return 如果需要旋转返回true
 */
bool PathController::shouldRotateToGoal(const geometry_msgs::PoseStamped& cur_pose, const geometry_msgs::PoseStamped& goal_pose)
{
    return std::hypot(cur_pose.pose.position.x - goal_pose.pose.position.x, cur_pose.pose.position.y - goal_pose.pose.position.y) < goal_dist_tolerance_;
}

/**
 * @brief 判断机器人是否需要旋转操作修正路径跟踪
 * @param angle_to_path 路径偏差角度
 * @param tolerance 角度偏差容忍值, 默认是0.0
 * @return 如果需要旋转返回true
 */
bool PathController::shouldRotateToPath(double angle_to_path, double tolerance)
{
    return (tolerance && (angle_to_path > tolerance)) || (!tolerance && (angle_to_path > rotate_tolerance_));
}

/**
 * @brief 线速度调节
 * @param base_odometry 机器人的里程计信息, 用于获取当前速度
 * @param v_d 期望的速度大小
 * @return v 调节后的线速度大小
 */
double PathController::linearRegularization(nav_msgs::Odometry& base_odometry, double v_d)
{
    if(std::fabs(v_d) > max_v_){
        v_d = std::copysign(max_v_, v_d);
    }

    double v = std::hypot(base_odometry.twist.twist.linear.x, base_odometry.twist.twist.linear.y);
    double v_inc = v_d - v;

    if(std::fabs(v_inc) > max_v_inc_){
        v_inc = std::copysign(max_v_inc_, v_inc);
    }

    double v_cmd = v + v_inc;
    if(std::fabs(v_cmd) > max_v_){
        v_cmd = std::copysign(max_v_, v_cmd);
    } else if(std::fabs(v_cmd) < min_v_){
        v_cmd = std::copysign(min_v_, v_cmd);
    }

    return v_cmd;
}

/**
 * @brief 角速度调节
 * @param base_odometry 机器人的里程计信息, 用于获取当前速度
 * @param w_d 期望的角速度
 * @return w 调节后的角速度
 */
double PathController::angularRegularization(nav_msgs::Odometry& base_odometry, double w_d)
{
    if(std::fabs(w_d) > max_w_){
        w_d = std::copysign(max_w_, w_d);
    }
    
    double w = base_odometry.twist.twist.angular.z;
    double w_inc = w_d - w;

    if(std::fabs(w_inc) > max_w_inc_){
        w_inc = std::copysign(max_w_inc_, w_inc);
    }

    double w_cmd = w + w_inc;
    if(std::fabs(w_cmd) > max_w_){
        w_cmd = std::copysign(max_w_, w_cmd);
    } else if(std::fabs(w_cmd) < min_w_){
        w_cmd = std::copysign(min_w_, w_cmd);
    }

    return w_cmd;
}

/**
 * @brief 使用tf将输入姿态转换到目标坐标系
 * @param tf tf2的缓冲区指针
 * @param out_frame 目标坐标系名称
 * @param in_pose 输入姿态
 * @param out_pose 转换后的姿态 
 */
void PathController::transformPose(tf2_ros::Buffer* tf, const std::string out_frame, const geometry_msgs::PoseStamped in_pose,
                   geometry_msgs::PoseStamped& out_pose) const
{
    if(in_pose.header.frame_id == out_frame){
        out_pose = in_pose;
    }

    tf->transform(in_pose, out_pose, out_frame);
    out_pose.header.frame_id = out_frame;
}

/**
 * @brief 将世界坐标系中的点(x, y)转换到代价地图坐标系中的点(x, y)
 * @param mx 代价地图中的x坐标
 * @param my 代价地图中的y坐标
 * @param wx 世界坐标系中的x坐标
 * @param wy 世界坐标字中的y坐标
 * @return 转换成功返回true
 */
bool PathController::worldToMap(double wx, double wy, int& mx, int& my)
{
    unsigned int mx_u, my_u;
    bool flag = costmap_ros_->getCostmap()->worldToMap(wx, wy, mx_u, my_u);
    mx = static_cast<int>(mx_u);
    my = static_cast<int>(my_u);

    return flag;
}

/**
 * @brief 移除机器人已经经过的路点和距离较远的路点
 * @param robot_pose_global 机器人当前的全局姿态
 * @return 移除后的路径
 */
std::vector<geometry_msgs::PoseStamped> PathController::removePoints(const geometry_msgs::PoseStamped robot_pose_global)
{   
    /**
     * @brief Lambda函数, 计算两个PoseStamped点之间的距离
     */
    auto calPoseDistance = [](const geometry_msgs::PoseStamped& ps_1, const geometry_msgs::PoseStamped& ps_2){
        return std::hypot(ps_1.pose.position.x - ps_2.pose.position.x, ps_1.pose.position.y - ps_2.pose.position.y);
    };

    /**
     * @brief Lambda函数, 查找路径中第一个累积距离大于某个阈值的点
     * @param begin 路径点的起始迭代器
     * @param end 路径点的结束迭代器
     * @param dist 计算距离函数
     * @param cmp_val 比较值 (累积距离的阈值)
     * @return 返回第一个累积距离大于阈值的路径点的迭代器
     */
    auto firstIntegrateDistance = [](std::vector<geometry_msgs::PoseStamped>::iterator begin, std::vector<geometry_msgs::PoseStamped>::iterator end,
                                     std::function<double(const geometry_msgs::PoseStamped&, const geometry_msgs::PoseStamped&)> dist,
                                     double cmp_val){
        if(begin == end) return end;
        double d = 0.0;
        for(auto it = begin; it != end; ++it){
            d += dist(*it, *(it + 1));
            if(d > cmp_val) return it + 1;
        }
        return end;
    };

    /**
     * @brief Lambda函数, 查找路径点上某个函数值最小的路径点
     * @param begin 路径点的起始迭代器
     * @param end 路径点的结束迭代器
     * @param cal 自定义计算函数, 用于计算路径点的某个值
     * @return 返回最小值对应路径点的最小值 
     */
    auto getMinFuncVal = [](std::vector<geometry_msgs::PoseStamped>::iterator begin, std::vector<geometry_msgs::PoseStamped>::iterator end,
                            std::function<double(const geometry_msgs::PoseStamped&)> cal){
        if(begin == end) return end;

        auto min_val = cal(*begin);
        auto min_iter = begin;
        for(auto it = ++begin; it != end; ++it){
            auto val = cal(*it);
            if(val <= min_val){
                min_val = val;
                min_iter = it;
            }
        }
        return min_iter;
    };

    // 确定地图的上界, 查找路径点中第一个累积距离超过代价地图宽度一半的路径点
    auto closet_pose_upper_bound = firstIntegrateDistance(global_plan_.begin(), global_plan_.end(), calPoseDistance, 
                                                          costmap_ros_->getCostmap()->getSizeInMetersX() / 2.0);
    
    // 在机器人上界范围内, 查找离机器人最近的路径点
    auto transform_begin = getMinFuncVal(global_plan_.begin(), closet_pose_upper_bound, 
                                         [&](const geometry_msgs::PoseStamped& ps){ return calPoseDistance(robot_pose_global, ps); });

    // 从最近的路径点开始, 提取后续的路径点
    std::vector<geometry_msgs::PoseStamped> remove_path;
    for(auto it = transform_begin; it < global_plan_.end(); ++it){
        remove_path.push_back(*it);
    }

    // 移除机器人已经经过的部分
    global_plan_.erase(std::begin(global_plan_), transform_begin);

    return remove_path;
}

/**
 * @brief 根据当前速度动态计算前瞻距离
 * @param vt 当前速度
 * @return dist 前瞻距离
 */
double PathController::getLookAheadDistance(double vt)
{
    double lookahead_dist = fabs(vt) * lookahead_time_;
    return common::math::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
}

/**
 * @brief 在路径上找到正好距离机器人是前瞻距离的点
 * @param lookahead_dist 前瞻距离
 * @param robot_pose_global 机器人当前的全局姿态
 * @param remove_plan 移除后的路径
 * @param pt 前瞻点
 * @param theta 轨迹上的角度
 * @param curvature 轨迹上的曲率
 */
double PathController::getLookAheadPoint(double lookahead_dist, geometry_msgs::PoseStamped robot_pose_global, 
                         const std::vector<geometry_msgs::PoseStamped>& remove_plan, geometry_msgs::PointStamped& pt,
                         double& theta, double& curvature)
{   // 获取机器人当前位置
    double rx = robot_pose_global.pose.position.x;
    double ry = robot_pose_global.pose.position.y;
    
    // 查找第一个距离大于前瞻距离的路径点  find_if遍历
    auto goal_pose_it = std::find_if(remove_plan.begin(), remove_plan.end(), [&](const geometry_msgs::PoseStamped& ps){
        return std::hypot(ps.pose.position.x - rx, ps.pose.position.y - ry) >= lookahead_dist;
    });
    
    // 路径中没有这样的点则选择最后一个路径点
    if(goal_pose_it == remove_plan.end()){
        goal_pose_it = std::prev(remove_plan.end());
        pt.point.x = goal_pose_it->pose.position.x;
        pt.point.y = goal_pose_it->pose.position.y;
        curvature = 0;
        theta = atan2(pt.point.y - ry, pt.point.x - rx);
    } else{
        // 前一个路径点的坐标
        double px, py;                                
        // 当前找到的路径点的坐标
        double gx = goal_pose_it->pose.position.x;    
        double gy = goal_pose_it->pose.position.y;
        if(goal_pose_it == remove_plan.begin()){
            px = rx;
            py = ry;
        } else{
            auto prev_pose_it = std::prev(goal_pose_it);
            px = prev_pose_it->pose.position.x;
            py = prev_pose_it->pose.position.y;
        }

        std::pair<double, double> prev_p(px - rx, py - ry);
        std::pair<double, double> goal_p(gx - rx, gy - ry);
        std::vector<std::pair<double, double>> i_points = common::math::circleSegmentIntersection(prev_p, goal_p, lookahead_dist);
        //将交点转换回全局坐标系
        pt.point.x = i_points[0].first + rx;    
        pt.point.y = i_points[0].second + ry;

        //计算曲率
        auto next_pose_it = std::next(goal_pose_it);
        if(next_pose_it != remove_plan.end()){
            double ax = px;
            double ay = py;
            double bx = gx;
            double by = gy;
            double cx = next_pose_it->pose.position.x;
            double cy = next_pose_it->pose.position.y;
            double a = std::hypot(bx - cx, by - cy);
            double b = std::hypot(cx - ax, cy - ay);
            double c = std::hypot(ax - bx, ay - by);

            double cosB = (a*a + c*c - b*b) / (2 * a * c);
            double sinB = std::sin(std::acos(cosB));

            double cross = (bx - ax) * (cy - ay) - (by -ay)* (cx - ax);
            curvature = std::copysign(2 * sinB / b, cross);
        } else{
            curvature = 0.0;
        }

        theta = atan2(gy - py, gx - px);
    }

    if(std::isnan(curvature)){
        curvature = 0.0;
    }
    pt.header.frame_id = goal_pose_it->header.frame_id;
    pt.header.stamp = goal_pose_it->header.stamp;
}

} // namespace path_controller

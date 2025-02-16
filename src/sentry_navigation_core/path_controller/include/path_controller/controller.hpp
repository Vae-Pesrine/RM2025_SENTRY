#ifndef PATH_CONTROLLER_CONTROLLER_HPP                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
#define PATH_CONTROLLER_CONTROLLER_HPP

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

namespace path_controller
{
class PathController
{
public:
    PathController();

    ~PathController();

    /**
     * @brief 设置或重置障碍物因子
     * @param factor 障碍物因子, 值越大表示障碍物的影响越大
     */
    void setFactor(double factor);

    /**
     * @brief 设置或重置坐标系名称
     * @param base_frame 机器人基坐标系名称
     * @param map_frame 地图坐标系名称
     */
    void setBaseFrame(std::string base_frame);
    void setmapFrame(std::string map_frame);

    /**
     * @brief 将角度标准化到 [-pi, pi]的范围内
     * @param angle 需要标准化的角度
     * @return reg_angle 标准化后的角度
     */
    double regularizeAngle(double angle);

    /**
     * @brief 从PoseStamped中获取偏航角
     * @param ps PoseStamped对象, 用于计算n偏航角
     * @return yaw 偏航角
     */
    double getYawAngle(geometry_msgs::PoseStamped& ps);

    /**
     * @brief 判断机器人是否需要通过旋转操作到达目标姿态
     * @param cur_pose 机器人当前姿态
     * @param goal_pose 机器人目标姿态
     * @return 如果需要旋转返回true
     */
    bool shouldRotateToGoal(const geometry_msgs::PoseStamped& cur_pose, const geometry_msgs::PoseStamped& goal_pose);

    /**
     * @brief 判断机器人是否需要旋转操作修正路径跟踪
     * @param angle_to_path 路径偏差角度
     * @param tolerance 角度偏差容忍值, 默认是0.0
     * @return 如果需要旋转返回true
     */
    bool shouldRotateToPath(double angle_to_path, double tolerance = 0.0);

    /**
     * @brief 线速度调节
     * @param base_odometry 机器人的里程计信息, 用于获取当前速度
     * @param v_d 期望的速度大小
     * @return v 调节后的线速度大小
     */
    double linearRegularization(nav_msgs::Odometry& base_odometry, double v_d);

    /**
     * @brief 角速度调节
     * @param base_odometry 机器人的里程计信息, 用于获取当前速度
     * @param w_d 期望的角速度
     * @return w 调节后的角速度
     */
    double angularRegularization(nav_msgs::Odometry& base_odometry, double w_d);

    /**
     * @brief 使用tf将输入姿态转换到目标坐标系
     * @param tf tf2的缓冲区指针
     * @param out_frame 目标坐标系名称
     * @param in_pose 输入姿态
     * @param out_pose 转换后的姿态 
     */
    void transformPose(tf2_ros::Buffer* tf, const std::string out_frame, const geometry_msgs::PoseStamped in_pose,
                       geometry_msgs::PoseStamped& out_pose) const;

    /**
     * @brief 将世界坐标系中的点(x, y)转换到代价地图坐标系中的点(x, y)
     * @param mx 代价地图中的x坐标
     * @param my 代价地图中的y坐标
     * @param wx 世界坐标系中的x坐标
     * @param wy 世界坐标字中的y坐标
     * @return 转换成功返回true
     */
    bool worldToMap(double wx, double wy, int& mx, int& my);

    /**
     * @brief 移除机器人已经经过的路点和距离较远的路点
     * @param robot_pose_global 机器人当前的全局姿态
     * @return 移除后的路径
     */
    std::vector<geometry_msgs::PoseStamped> removePoints(const geometry_msgs::PoseStamped robot_pose_global);

    /**
     * @brief 根据当前速度动态计算前瞻距离
     * @param vt 当前速度
     * @return dist 前瞻距离
     */
    double getLookAheadDistance(double vt);

    /**
     * @brief 在路径上找到正好距离机器人是前瞻距离的点
     * @param lookahead_dist 前瞻距离
     * @param robot_pose_global 机器人当前的全局姿态
     * @param remove_plan 移除后的路径
     * @param pt 前瞻点
     * @param theta 轨迹上的角度
     * @param curvature 轨迹上的曲率
     */
    double getLookAheadPoint(double lookahead_dist, geometry_msgs::PoseStamped robot_pose_global, 
                             const std::vector<geometry_msgs::PoseStamped>& remove_plan, geometry_msgs::PointStamped& pt,
                             double& theta, double& curvature);
protected:
    double factor_;                     //障碍物因子

    double max_v_, min_v_, max_v_inc_;    // 线速度的最大值, 最小值, 最大增量
    double max_w_, min_w_, max_w_inc_;    // 角速度的最大值, 最小值, 最大增量

    double goal_dist_tolerance_;        // 距离的容差值
 
    double rotate_tolerance_;           // 角度的容差值

    std::string base_frame_, map_frame_, odom_frame_;  // 机器人基坐标系, 地图坐标系, 里程计坐标系

    std::shared_ptr<base_local_planner::OdometryHelperRos> odom_helper_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;  // 全局路径

    double lookahead_time_;
    double min_lookahead_dist_;
    double max_lookahead_dist_;
};
}





#endif
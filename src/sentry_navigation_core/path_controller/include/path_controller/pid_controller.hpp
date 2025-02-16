#ifndef PATH_CONTROLLER_PID_CONTROLLER_HPP_
#define PATH_CONTROLLER_PID_CONTROLLER_HPP_

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>

#include "path_controller/controller.hpp"

namespace path_controller
{
class PIDController: public nav_core::BaseLocalPlanner, PathController
{
public:
    PIDController();

    PIDController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    ~PIDController();
    
    /**
     * @brief 初始化局部控制器
     * @param name 控制器名称
     * @param tf 变换监听器
     * @param costmap_ros 为轨迹分配代价的代价地图
     */
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief 设置控制器正在跟踪的路径
     * @param ori_global_plan 要传给控制器的路径
     * @return 路径更新成功返回true
     */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& ori_global_plan);

    /**
     * @brief 检查是否达到目标状态
     * @return 达到目标姿态返回true
     */
    bool isGoalReached();

    /**
     * @brief 根据机器人当前的位置,方向和速度计算速度指令
     * @param cmd_vel 将被传送给机器人底盘的速度指令
     * @return 找到有效的轨迹返回true 
     */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

private:
    bool initialized_;              // 初始化标志
    bool goal_reached_;             // 目标是否达到的标志
    tf2_ros::Buffer* tf_;

    double k_v_p_, k_v_i_, k_v_d_;  // 线速度的比例, 积分, 微分增益
    double k_w_p_, k_w_i_, k_w_d_;  // 角速度的比例, 积分, 微分增益
    double k_theta_;                // 偏航角增益
    double k_, l_;                  

    double e_v_, e_w_;              // 线速度和角速度的误差
    double i_v_, i_w_;              // 线速度和角速度的积分项

    double controller_frequency_;   // 控制频率 
    double d_t_;                    // 控制时间步长

    ros::Publisher pub_target_pose_;
    ros::Publisher pub_current_pose_;

    double goal_x_, goal_y_, goal_theta_; // 目标参数

    Eigen::Vector2d _pidControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r);

};
} // namespace path_controller

#endif
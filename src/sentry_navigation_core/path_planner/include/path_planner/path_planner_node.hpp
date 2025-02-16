#ifndef PATH_PLANNER_PATH_PLANNER_NODE_HPP_
#define PATH_PLANNER_PATH_PLANNER_NODE_HPP_

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>

#include "motion_planning/geometry/point.hpp"
#include "path_planner/path_planner.hpp"

namespace path_planner
{
class PathPlannerNode: public nav_core::BaseGlobalPlanner
{
public:
    /**
     * @brief 构造一个新的路径规划器对象
     */
    PathPlannerNode();
    
    /**
     * @brief 构造一个新的路径规划器对象
     * @param name        规划器名称
     * @param costmap_ros 用于路径规划的代价地图（ROS 封装）
     */
    PathPlannerNode(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    ~PathPlannerNode() = default;

    /**
     * @brief 规划器初始化
     * @param name       规划器名称
     * @param costmapRos 代价地图的 ROS 封装
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos);

    /**
     * @brief 规划器初始化
     * @param name 规划器名称
     */
    void initialize(std::string name);
    /**
     * @brief 在世界地图中规划路径
     * @param start 起始点（世界地图坐标）
     * @param goal  目标点（世界地图坐标）
     * @param plan  规划的路径
     * @return 如果成功找到路径返回 true，否则返回 false
     */
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    /**
     * @brief 在世界地图中规划路径
     * @param start 起始点（世界地图坐标）
     * @param goal  目标点（世界地图坐标）
     * @param tolerance 目标容忍度
     * @param plan  规划的路径
     * @return 如果成功找到路径返回 true，否则返回 false
     */
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                std::vector<geometry_msgs::PoseStamped>& plan);
    /**
     * @brief 注册规划服务
     * @param req  客户端请求
     * @param resp 服务器响应
     */
    bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

protected:
    bool _getPlanFromPath(PathPlanner::Points3d& path, std::vector<geometry_msgs::PoseStamped>& plan);

protected:
    enum PLANNER_TYPE
    {
        GRAPH_PLANNER = 0,  // 图形规划器
        SAMPLE_PLANNER = 1, // 采样规划器
    };

protected: 
    bool initialized_;                           // 初始化标志
    costmap_2d::Costmap2DROS* costmap_ros_;      // 代价地图的ROS封装
    std::string frame_id_;                       // 代价地图的ID
    std::string planner_name_;                   // 规划器名称
    PLANNER_TYPE planner_type_;                  // 规划器类型

    std::string smoother_name_;                  // 平滑器名称
    std::shared_ptr<PathPlanner> global_planner_;     // 全局图形规划器

    ros::Publisher pub_plan_;                    // 路径规划发布器
    ros::Publisher pub_expand_;                  // 节点扩展发布器
    ros::Publisher pub_tree_;                    // 随机搜索树发布器
    ros::Publisher pub_particles_;               // 进化粒子发布器
    ros::ServiceServer make_plan_srv_;           // 规划服务

private:
    bool is_outline_;   // 是否绘制地图边界
    bool is_expand_;    // 是否发布扩展地图
    
    double tolerance_;  // 容忍度
    double factor_;     // 障碍物膨胀因子
};

} // namespace path_planner
#endif
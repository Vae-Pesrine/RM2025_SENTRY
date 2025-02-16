#ifndef PATH_PLANNER_GRAPH_PLANNER_A_STAR_HPP_
#define PATH_PLANNER_GRAPH_PLANNER_A_STAR_HPP_

#include "path_planner/path_planner.hpp"

namespace path_planner
{
/**
 * @brief A* 算法的实现类
 */
class AStarPathPlanner: public PathPlanner
{
public:
    /**
     * @brief 构造一个新的 AStar 对象
     * @param costmap_ros 用于路径规划的环境（代价地图的 ROS 封装）
     * @param dijkstra 是否使用 Dijkstra 算法实现
     * @param gbfs 是否使用贪婪最佳优先搜索（GBFS）实现
     */
    AStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, bool dijkstra = false, bool gbfs = false);
    
    /**
     * @brief A* 算法的实现
     * @param start 起始节点
     * @param goal 目标节点
     * @param path 最优路径，由 Node 组成
     * @param expand 在搜索过程中被搜索的节点
     * @return 如果找到路径返回 true，否则返回 false
     */
    bool plan(const Point3d& start, const Point3d& goal, Points3d& payh, Points3d& expand);


private:
    bool is_dijkstra_;    // 是否使用 Dijkstra 算法
    bool is_gbfs_;        // 是否使用贪婪最佳优先搜索（GBFS）

    using Node = common::structure::Node<int>;
    const std::vector<Node> motions = {
        { 0, 1, 1.0 },          { 1, 0, 1.0 },           { 0, -1, 1.0 },          { -1, 0, 1.0 },
        { 1, 1, std::sqrt(2) }, { 1, -1, std::sqrt(2) }, { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
    };                    // 定义了八个可能的运动方向，包括上下左右和四个对角线方向
};
} // namespace path_planner






#endif
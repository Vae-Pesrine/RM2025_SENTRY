#include <queue>
#include <vector>
#include <unordered_set>

#include <costmap_2d/cost_values.h>
#include <path_planner/graph_planner/astar_planner.hpp>

namespace path_planner
{
/**
 * @brief 构造一个新的 AStar 对象
 * @param costmap_ros 用于路径规划的环境（代价地图的 ROS 封装）
 * @param dijkstra 是否使用 Dijkstra 算法实现
 * @param gbfs 是否使用贪婪最佳优先搜索（GBFS）实现
 */
AStarPathPlanner::AStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, bool dijkstra, bool gbfs)
    : PathPlanner(costmap_ros)
{
    if(!(dijkstra && gbfs)){
        is_dijkstra_ = dijkstra;
        is_gbfs_ = gbfs;
    } else{
        is_dijkstra_ = false;
        is_gbfs_ = false;
    }
};

/**
 * @brief A* 算法的实现
 * @param start 起始节点
 * @param goal 目标节点
 * @param path 最优路径，由 Node 组成
 * @param expand 在搜索过程中被搜索的节点
 * @return 如果找到路径返回 true，否则返回 false
 */
bool AStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  // 初始化起始节点和目标节点
  Node start_node(start.x(), start.y());
  Node goal_node(goal.x(), goal.y());
  start_node.set_id(grid2Index(start_node.x(), start_node.y()));
  goal_node.set_id(grid2Index(goal_node.x(), goal_node.y()));

  // 清空路径和扩展节点集合
  path.clear();
  expand.clear();

  // 初始化开放列表和关闭列表
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  // 将起始节点加入开放列表
  open_list.push(start_node);

  // 主要规划过程
  while (!open_list.empty()){
    // 从开放列表中取出当前节点
    auto current = open_list.top();
    open_list.pop();

    // 如果当前节点已经在关闭列表中，则跳过
    if (closed_list.find(current.id()) != closed_list.end())
      continue;

    // 将当前节点加入关闭列表，并记录被搜索的节点
    closed_list.insert(std::make_pair(current.id(), current));
    expand.emplace_back(current.x(), current.y());

    // 如果当前节点是目标节点，则找到路径
    if (current == goal_node){
      // 从关闭列表中回溯路径
      const auto& backtrace = _convertClosedListToPath<Node>(closed_list, start_node, goal_node);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter){
        path.emplace_back(iter->x(), iter->y());
      }
      return true;
    }

    // 探索当前节点的近邻节点
    for (const auto& motion : motions){
      // 探索一个新的节点
      auto node_new = current + motion;
      node_new.set_g(current.g() + motion.g());
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));

      // 如果新节点已经在关闭列表中，则跳过
      if (closed_list.find(node_new.id()) != closed_list.end())
        continue;

      node_new.set_pid(current.id());

      // 如果新节点超出边界或遇到障碍物，则跳过
      if ((node_new.id() < 0) || (node_new.id() >= map_size_) ||
          (costmap_->getCharMap()[node_new.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
           costmap_->getCharMap()[node_new.id()] >= costmap_->getCharMap()[current.id()]))
        continue;

      // 如果使用 Dijkstra 实现，则不考虑启发式成本
      if (!is_dijkstra_)
        node_new.set_h(std::hypot(node_new.x() - goal_node.x(), node_new.y() - goal_node.y()));

      // 如果使用 GBFS 实现，则只考虑启发式成本
      if (is_gbfs_)
        node_new.set_g(0.0);
      // 否则，通过 node_new = current + m 计算 g 值

      open_list.push(node_new);
    }
  }

  return false;
}
}
#include <iostream>
#include <queue>
#include <unordered_set>

#include "motion_planning/math/math_tools.hpp" 
#include "path_planner/graph_planner/hybrid_astar_planner.hpp"

namespace path_planner
{
namespace
{
    constexpr double kPenaltyTurning = 1.05;  // 转弯惩罚
    constexpr double kPenaltyCod = 1.5;      // 方向变化惩罚
    constexpr double kPenaltyReversing = 1.5; // 后退惩罚
    constexpr int kHeadings = 72;            // 朝向数量
    constexpr double kDeltaHeading = (2 * M_PI / kHeadings); // 朝向间隔

    // 车辆半径
    constexpr double R = 1.3;
    
    // 转弯角度
    constexpr double alpha = 14 * M_PI / 180;
    
    const std::vector<HybridAStarPathPlanner::Node> motions = {
      { 0, 1, 1.0 },          { 1, 0, 1.0 },           { 0, -1, 1.0 },          { -1, 0, 1.0 },
      { 1, 1, std::sqrt(2) }, { 1, -1, std::sqrt(2) }, { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
    };
}  // namespace

/**
 * @brief 3D 节点类的构造函数
 * @param x   x 值
 * @param y   y 值
 * @param t   朝向角
 * @param g   g 值，到达该节点的成本
 * @param h   h 值，该节点的启发式成本
 * @param id  节点的 ID
 * @param pid 父节点的 ID
 * @param prim 
 */
HybridAStarPathPlanner::HybridNode::HybridNode(double x, double y, double t, double g, double h, int id, int pid,
                                               int prim)
  : Node(x, y, g, h, id, pid), theta_(t), prim_(prim)
{
}

double HybridAStarPathPlanner::HybridNode::theta() const
{
  return theta_;
}

void HybridAStarPathPlanner::HybridNode::set_theta(double theta)
{
  theta_ = theta;
}

/**
 * @brief 重载 + 操作符
 * @param n 另一个节点
 * @return 当前节点和输入节点 n 的值相加后的节点
 */
HybridAStarPathPlanner::HybridNode HybridAStarPathPlanner::HybridNode::operator+(const HybridNode& n) const
{
  HybridNode result;

  result.x_ = x_ + n.x_ * cos(theta_) - n.y_ * sin(theta_);
  result.y_ = y_ + n.x_ * sin(theta_) + n.y_ * cos(theta_);
  result.theta_ = common::math::mod2pi(theta_ + n.theta_);
  result.prim_ = n.prim_;

  // 前进驾驶
  if (prim_ < 3){
    // 惩罚转弯
    if (n.prim_ != prim_){
      // 惩罚方向变化
      if (n.prim_ > 2)
        result.set_g(g() + n.x_ * kPenaltyTurning * kPenaltyCod);
      else
        result.set_g(g() + n.x_ * kPenaltyTurning);
    } else
      result.set_g(g() + n.x_);
  }
  // 后退驾驶
  else{
    // 惩罚转弯和后退
    if (n.prim_ != prim_){
      // 惩罚方向变化
      if (n.prim_ < 3)
        result.set_g(g() + n.x_ * kPenaltyTurning * kPenaltyCod * kPenaltyReversing);
      else
        result.set_g(g() + n.x_ * kPenaltyTurning * kPenaltyReversing);
    } else
      result.set_g(g() + n.x_ * kPenaltyReversing);
  }

  return result;
}

/**
 * @brief 重载 == 操作符
 * @param n 另一个节点
 * @return 如果当前节点等于节点 n，则返回 true，否则返回 false
 */
bool HybridAStarPathPlanner::HybridNode::operator==(const HybridNode& n) const
{
  return (x_ == n.x_) && (y_ == n.y_) &&
         ((std::abs(theta_ - n.theta_) <= kDeltaHeading) ||
          (std::abs(theta_ - n.theta_) >= (2 * M_PI - kDeltaHeading)));
}

/**
 * @brief 重载 != 操作符
 * @param n 另一个节点
 * @return 如果当前节点不等于节点 n，则返回 true，否则返回 false
 */
bool HybridAStarPathPlanner::HybridNode::operator!=(const HybridNode& n) const
{
  return !operator==(n);
}

/**
 * @brief 获取允许的运动
 * @return 节点向量，包含允许的运动
 */
std::vector<HybridAStarPathPlanner::HybridNode> HybridAStarPathPlanner::HybridNode::getMotion()
{
  // R, alpha
  double dy[] = { 0, -R * (1 - cos(alpha)), R * (1 - cos(alpha)) };
  double dx[] = { alpha * R, R * sin(alpha), R * sin(alpha) };
  double dt[] = { 0, alpha, -alpha };

  return {
    HybridNode(dx[0], dy[0], dt[0], 0, 0, 0, 0, 0),   HybridNode(dx[1], dy[1], dt[1], 0, 0, 0, 0, 1),
    HybridNode(dx[2], dy[2], dt[2], 0, 0, 0, 0, 2),   HybridNode(-dx[0], dy[0], -dt[0], 0, 0, 0, 0, 3),
    HybridNode(-dx[1], dy[1], -dt[1], 0, 0, 0, 0, 4), HybridNode(-dx[2], dy[2], -dt[2], 0, 0, 0, 0, 5),
  };
}

/**
 * @brief 构造一个新的混合 A* 对象
 * @param costmap_ros 用于路径规划的环境（代价地图的 ROS 封装）
 * @param is_reverse 是否允许后退操作
 * @param max_curv 模型的最大曲率
 */
HybridAStarPathPlanner::HybridAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, bool is_reverse, double max_curv)
  : PathPlanner(costmap_ros)
  , is_reverse_(is_reverse)
  , max_curv_(max_curv)
  , dubins_gen_(std::make_unique<common::geometry::DubinsCurve>(1.5, max_curv))
  , a_star_planner_(std::make_unique<AStarPathPlanner>(costmap_ros))
  , goal_(HybridNode())
{
}

/**
 * @brief 混合 A* 算法的实现
 * @param start 起始节点
 * @param goal 目标节点
 * @param path 由 Node 组成的最优路径
 * @param expand 在搜索过程中被搜索的节点
 * @return 如果找到路径返回 true，否则返回 false
 */
bool HybridAStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  // 初始化
  path.clear();
  expand.clear();
  HybridNode start_node(start.x(), start.y(), start.theta());
  HybridNode goal_node(goal.x(), goal.y(), goal.theta());
  updateIndex(start_node);
  updateIndex(goal_node);

  // 更新启发式地图
  if (goal_ != goal_node){
    goal_.set_x(goal.x());
    goal_.set_y(goal.y());
    goal_.set_theta(goal.theta());
    double gx, gy;
    world2Map(goal.x(), goal.y(), gx, gy);
    HybridNode h_start(gx, gy, 0, 0, 0, grid2Index(gx, gy), 0);
    genHeurisiticMap(h_start);
  }

  // 可能的方向和运动
  int dir = is_reverse_ ? 6 : 3;
  const std::vector<HybridNode> motions = HybridNode::getMotion();

  // 开放列表和关闭列表
  std::priority_queue<HybridNode, std::vector<HybridNode>, HybridNode::compare_cost> open_list;
  std::unordered_map<int, HybridNode> closed_list;

  open_list.push(start_node);

  // 主要过程
  while (!open_list.empty()){
    // 从开放列表中取出当前节点
    HybridNode current = open_list.top();
    open_list.pop();

    // 当前节点是否已在关闭列表中
    if (closed_list.find(current.id()) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current.id(), current));
    expand.emplace_back(current.x(), current.y());

    // 目标曲线连接
    std::vector<HybridNode> path_dubins;
    if (std::hypot(current.x() - goal.x(), current.y() - goal.y()) < 50){
      if (dubinsShot(current, goal_node, path_dubins)){
        const auto& backtrace = _convertClosedListToPath(closed_list, start_node, current);
        for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter){
          path.emplace_back(iter->x(), iter->y());
        }
        for (const auto& dubins_pt : path_dubins){
          path.emplace_back(dubins_pt.x(), dubins_pt.y());
        }
        return true;
      }
    }

    // 探索当前节点的近邻 s
    for (size_t i = 0; i < dir; ++i){
      // 探索一个新节点
      HybridNode node_new = current + motions[i];
      updateIndex(node_new);

      // 新节点是否已在关闭列表中
      if (closed_list.find(node_new.id()) != closed_list.end())
        continue;

      // 下一个节点是否超出边界或遇到障碍物
      // 防止在当前节点在膨胀区域内时规划失败
      if ((_worldToIndex(node_new.x(), node_new.y()) < 0) || (_worldToIndex(node_new.x(), node_new.y()) >= map_size_) ||
          (node_new.theta() / kDeltaHeading >= kHeadings) ||
          (costmap_->getCharMap()[_worldToIndex(node_new.x(), node_new.y())] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
           costmap_->getCharMap()[_worldToIndex(node_new.x(), node_new.y())] >=
               costmap_->getCharMap()[_worldToIndex(current.x(), current.y())]))
        continue;

      node_new.set_pid(current.id());
      updateHeuristic(node_new);

      open_list.push(node_new);
    }
  }

  // 候选 A* 路径
  return a_star_planner_->plan(start, goal, path, expand);
}

/**
 * @brief 尝试使用 Dubins 曲线连接起始点和目标点
 * @param start 起始节点
 * @param goal 目标节点
 * @param path 起始点和目标点之间的 Dubins 路径
 * @return 如果射击成功返回 true，否则返回 false
 */
bool HybridAStarPathPlanner::dubinsShot(const HybridNode& start, const HybridNode& goal, std::vector<HybridNode>& path)
{
  double sx, sy, gx, gy;
  world2Map(start.x(), start.y(), sx, sy);
  world2Map(goal.x(), goal.y(), gx, gy);
  common::geometry::Points3d poes = { { sx, sy, start.theta() }, { gx, gy, goal.theta() } };
  common::geometry::Points2d path_dubins;

  if (dubins_gen_->run(poes, path_dubins)){
    path.clear();
    for (auto const& p : path_dubins){
      if (costmap_->getCharMap()[grid2Index(p.x(), p.y())] >= costmap_2d::LETHAL_OBSTACLE * factor_)
        return false;
      else
        path.emplace_back(p.x(), p.y());
    }
    return true;
  }
  else
    return false;
}

/**
 * @brief 更新混合节点的索引
 * @param node 要更新的混合节点
 */
void HybridAStarPathPlanner::updateIndex(HybridNode& node)
{
  node.set_id(static_cast<int>(node.theta() / kDeltaHeading) + _worldToIndex(node.x(), node.y()));
}

/**
 * @brief 更新混合节点的 h 值
 * @param node 要更新的混合节点
 */
void HybridAStarPathPlanner::updateHeuristic(HybridNode& node)
{
  // Dubins 成本函数
  double cost_dubins = 0.0;

  // 2D 搜索成本函数
  double cost_2d = h_map_[_worldToIndex(node.x(), node.y())].g() * costmap_->getResolution();
  node.set_h(std::max(cost_2d, cost_dubins));
}

/**
 * @brief 使用 A* 算法生成启发式地图，地图中的每个值是它与起始点之间的距离。
 * @param start 起始节点
 */
void HybridAStarPathPlanner::genHeurisiticMap(const Node& start)
{
  // 开放列表和关闭列表
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> open_set;

  open_list.push(start);
  open_set.emplace(start.id(), start);

  // 主要过程
  while (!open_list.empty()){
    // 从开放列表中取出当前节点
    auto current = open_list.top();
    open_list.pop();

    h_map_.emplace(current.id(), current);

    // 探索当前节点的近邻
    for (const auto& motion : motions){
      // 探索一个新节点
      auto node_new = current + motion;
      node_new.set_g(current.g() + motion.g());
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));

      // 新节点是否已在关闭列表中
      if (h_map_.find(node_new.id()) != h_map_.end())
        continue;

      // 下一个节点是否超出边界或遇到障碍物
      // 防止在当前节点在膨胀区域内时规划失败
      if ((node_new.id() < 0) || (node_new.id() >= map_size_))
        continue;

      if (open_set.find(node_new.id()) != open_set.end()){
        if (open_set[node_new.id()].g() > node_new.g())
          open_set[node_new.id()].set_g(node_new.g());
      } else{
        open_list.push(node_new);
        open_set.emplace(node_new.id(), node_new);
      }
    }
  }
}

/**
 * @brief 将世界地图坐标 (x, y) 转换为网格索引 (i)
 * @param wx 世界地图 x 坐标
 * @param wy 世界地图 y 坐标
 * @return 索引
 */
int HybridAStarPathPlanner::_worldToIndex(double wx, double wy)
{
  double gx, gy;
  world2Map(wx, wy, gx, gy);
  return grid2Index(gx, gy);
}

/**
 * @brief 将关闭列表转换为路径
 * @param closed_list 关闭列表
 * @param start 起始节点
 * @param goal 目标节点
 * @return 包含路径节点的向量
 */
std::vector<HybridAStarPathPlanner::HybridNode> HybridAStarPathPlanner::_convertClosedListToPath(
    std::unordered_map<int, HybridNode>& closed_list, const HybridNode& start, const HybridNode& goal)
{
  double cur_x, cur_y;
  std::vector<HybridNode> path;
  auto current = closed_list.find(goal.id());
  while (current->second != start){
    world2Map(current->second.x(), current->second.y(), cur_x, cur_y);
    path.emplace_back(cur_x, cur_y);
    auto it = closed_list.find(current->second.pid());
    if (it != closed_list.end())
      current = it;
    else
      return {};
  }
  world2Map(start.x(), start.y(), cur_x, cur_y);
  path.emplace_back(cur_x, cur_y);
  return path;
}

} // namespace path_planner
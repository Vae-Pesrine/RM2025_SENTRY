#ifndef PATH_PLANNER_GRAPH_PLANNER_HYBRID_A_STAR_HPP_
#define PATH_PLANNER_GRAPH_PLANNER_HYBRID_A_STAR_HPP_

#include "motion_planning/geometry/dubins_curve.hpp"
#include "path_planner/path_planner.hpp"
#include "path_planner/graph_planner/astar_planner.hpp" 

namespace path_planner
{
class HybridAStarPathPlanner: public PathPlanner
{
public:
    using Node = common::structure::Node<double>;
    class HybridNode : public Node
    {
    public:
        /**
         * @brief HybridNode 类的构造函数
         * @param x   x 值
         * @param y   y 值
         * @param theta 朝向角
         * @param g   g 值，到达该节点的成本
         * @param h   h 值，该节点的启发式成本
         * @param id  节点的 ID
         * @param pid 父节点的 ID
         * @param prim 
         */
        HybridNode(double x = 0, double y = 0, double theta = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0,
                   int prim = 0);

        double theta() const;  // 获取朝向角
        void set_theta(double theta);  // 设置朝向角

        /**
         * @brief 重载 + 操作符
         * @param n 另一个 HybridNode
         * @return 当前节点和输入节点 n 的值相加后的节点
         */
        HybridNode operator+(const HybridNode& n) const;

        /**
         * @brief 重载 == 操作符
         * @param n 另一个 HybridNode
         * @return 如果当前节点等于节点 n，则返回 true，否则返回 false
         */
        bool operator==(const HybridNode& n) const;

        /**
         * @brief 重载 != 操作符
         * @param n 另一个 HybridNode
         * @return 如果当前节点不等于节点 n，则返回 true，否则返回 false
         */
        bool operator!=(const HybridNode& n) const;

        /**
         * @brief 获取允许的运动
         * @return HybridNode 向量，包含允许的运动
         */
        static std::vector<HybridNode> getMotion();

    private:
        double theta_;  // 朝向角
        int prim_;  
    };

/**
 * @brief 构造一个新的hybrid A* 对象
 * @param costmap_ros 用于路径规划的环境（代价地图的 ROS 封装）
 * @param is_reverse 是否允许后退操作
 * @param max_curv 模型的最大曲率
 */
HybridAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, bool is_reverse, double max_curv);

/**
 * @brief hybrid A* 对象
 */
~HybridAStarPathPlanner() = default;

/**
 * @brief hybrid A* 算法的实现
 * @param start 起始节点
 * @param goal 目标节点
 * @param path 由 Node 组成的最优路径
 * @param expand 在搜索过程中被搜索的节点
 * @return 如果找到路径返回 true，否则返回 false
 */
bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

/**
 * @brief 尝试使用 Dubins 曲线连接起始点和目标点
 * @param start 起始节点
 * @param goal 目标节点
 * @param path 起始点和目标点之间的 Dubins 路径
 * @return 如果连接成功返回 true，否则返回 false
 */
bool dubinsShot(const HybridNode& start, const HybridNode& goal, std::vector<HybridNode>& path);

/**
 * @brief 更新索引
 * @param node 要更新的节点
 */
void updateIndex(HybridNode& node);

/**
 * @brief 更新节点的 h 值
 * @param node 要更新的节点
 */
void updateHeuristic(HybridNode& node);

/**
 * @brief 使用 A* 算法生成启发式地图，地图中的每个值是它与起始点之间的距离。
 * @param start 起始节点
 */
void genHeurisiticMap(const Node& start);

protected:
  /**
   * @brief 将世界地图坐标 (x, y) 转换为网格索引 (i)
   * @param wx 世界地图 x 坐标
   * @param wy 世界地图 y 坐标
   * @return 索引
   */
  int _worldToIndex(double wx, double wy);

  /**
   * @brief 将关闭列表转换为路径
   * @param closed_list 关闭列表
   * @param start 起始节点
   * @param goal 目标节点
   * @return 包含路径节点的向量
   */
  std::vector<HybridNode> _convertClosedListToPath(std::unordered_map<int, HybridNode>& closed_list,
                                                   const HybridNode& start, const HybridNode& goal);


private:
    HybridNode goal_;                                             // 历史目标点
    std::unordered_map<int, Node> h_map_;                         // 启发式地图
    bool is_reverse_;                                             // 是否允许后退操作
    double max_curv_;                                             // 模型的最大曲率
    std::unique_ptr<AStarPathPlanner> a_star_planner_;             // A* 规划器
    std::unique_ptr<common::geometry::DubinsCurve> dubins_gen_;   // dubins 曲线生成器
};
} 

#endif
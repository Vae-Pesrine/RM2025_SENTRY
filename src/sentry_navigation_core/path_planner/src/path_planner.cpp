#include <costmap_2d/cost_values.h>
#include "path_planner/path_planner.hpp"

namespace path_planner
{
/**
 * @brief 构造一个新的全局路径规划器对象
 * @param costmap_ros 用于路径规划的环境（代价地图的 ROS 封装）
 */
PathPlanner::PathPlanner(costmap_2d::Costmap2DROS* costmap_ros)
    : factor_(0.5f), map_size_(0), costmap_ros_(costmap_ros), costmap_(costmap_ros->getCostmap())
{
    map_size_ = static_cast<int>(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY());
}

/**
 * @brief 设置或重置障碍物因子
 * @param factor 障碍物因子
 */
void PathPlanner::setFactor(float factor) 
{
    factor_ = factor;
}

/**
 * @brief 获取代价地图
 * @return costmap 代价地图指针
 */
costmap_2d::Costmap2D* PathPlanner::getCostMap() const
{
    return costmap_;
}

/**
 * @brief 获取代价地图的大小
 * @return map_size 代价地图的大小
 */
int PathPlanner::getMapSize() const
{
    return map_size_;
}

/**
 * @brief 将网格地图坐标 (x, y) 转换为网格索引 (i)
 * @param x 网格地图 x 坐标
 * @param y 网格地图 y 坐标
 * @return 索引
 */
int PathPlanner::grid2Index(int x, int y)
{
    return x + static_cast<int>(costmap_->getSizeInCellsX() * y);
}


/**
 * @brief 将网格索引 (i) 转换为网格地图坐标 (x, y)
 * @param i 网格索引
 * @param x 网格地图 x 坐标
 * @param y 网格地图 y 坐标
 */
void PathPlanner::index2Grid(int i, int& x, int& y)
{
    x = static_cast<int>(i % costmap_->getSizeInCellsX());
    y = static_cast<int>(i / costmap_->getSizeInCellsX());
}

/**
 * @brief 将世界地图坐标 (wx, wy) 转换为代价地图坐标 (mx, my)
 * @param mx 代价地图 x 坐标
 * @param my 代价地图 y 坐标
 * @param wx 世界地图 x 坐标
 * @param wy 世界地图 y 坐标
 * @return 如果转换成功返回 true，否则返回 false
 */
bool PathPlanner::world2Map(double wx, double wy, double& mx, double& my)
{
    if(wx < costmap_->getOriginX() || wy < costmap_->getOriginY()){
        return false;
    }

    mx = (wx - costmap_->getOriginX()) / costmap_->getResolution();
    my = (wy - costmap_->getOriginX()) / costmap_->getResolution();

    if(mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()){
        return true;
    }

    return false;
}

/**
 * @brief 将代价地图坐标 (mx, my) 转换为世界地图坐标 (wx, wy)
 * @param mx 代价地图 x 坐标
 * @param my 代价地图 y 坐标
 * @param wx 世界地图 x 坐标
 * @param wy 世界地图 y 坐标
 */
void PathPlanner::map2World(double mx, double my, double& wx, double& wy)
{
    wx = costmap_->getOriginX() + (mx + 0.5) * costmap_->getResolution();
    wy = costmap_->getOriginX() + (my + 0.5) * costmap_->getResolution();
}

/**
 * @brief 将代价地图的边界扩展为障碍物，以防止交叉规划
 */
void PathPlanner::outlineMap()
{
    auto nx = costmap_->getSizeInCellsX();
    auto ny = costmap_->getSizeInCellsY();
    auto pc = costmap_->getCharMap();
    for (int i = 0; i < nx; i++)
      *pc++ = costmap_2d::LETHAL_OBSTACLE;
    pc = costmap_->getCharMap() + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
      *pc++ = costmap_2d::LETHAL_OBSTACLE;
    pc = costmap_->getCharMap();
    for (int i = 0; i < ny; i++, pc += nx)
      *pc = costmap_2d::LETHAL_OBSTACLE;
    pc = costmap_->getCharMap() + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
      *pc = costmap_2d::LETHAL_OBSTACLE;
}
} // namespace path_planner
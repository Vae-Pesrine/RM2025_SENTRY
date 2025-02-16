#ifndef PATH_PLANNER_PATH_PLANNER_HPP_
#define PATH_PLANNER_PATH_PLANNER_HPP_

#include <unordered_map>

#include "ros/ros.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "motion_planning/structure/node.hpp"
#include "motion_planning/geometry/point.hpp"

namespace path_planner
{
class PathPlanner
{
public:
    using Point2d = common::geometry::Point2d;
    using Points2d = common::geometry::Points2d;
    using Point3d = common::geometry::Point3d;
    using Points3d = common::geometry::Points3d;

public:
    PathPlanner();

    PathPlanner(costmap_2d::Costmap2DROS* costmap_ros);
    
    virtual ~PathPlanner() = default;

    virtual bool plan(const Point3d& start, const Point3d& goal, Points3d path, Points3d& expand);

    void setFactor(float factor);

    costmap_2d::Costmap2D* getCostMap() const;

    int getMapSize() const;

    int grid2Index(int x, int y);
    
    void index2Grid(int i, int& x, int& y);

    bool world2Map(double wx, double wy, double& mx, double& my);

    void map2World(double mx, double my, double& wx, double& wy);

    void outlineMap();

protected:
    template <typename Node>
    std::vector<Node> _convertClosedListToPath(std::unordered_map<int, Node>& closed_list, const Node& start, const Node& goal)
    {
        std::vector<Node> path;
        auto current = closed_list.find(goal.id());
        while(current->second != start){
            path.emplace_back(current->second.x(), current->second.y());
            auto it = closed_list.find(current->second.pid());
            if(it != closed_list.end()){
                current = it;
            } else{
                return {};
            }
        }
        path.push_back(start);
    
        return path;
    }

    template <typename Node>
    std::vector<Node> _convertBiClosedListToPath(std::unordered_map<int, Node>& f_closed_list,
                                                 std::unordered_map<int, Node>& b_closed_list,
                                                 const Node& start, const Node& goal, const Node& boundary)
    {
        if(f_closed_list.find(start.id()) == f_closed_list.end()){
            std:swap(f_closed_list, b_closed_list);
        }

        std::vector<Node> path, path_b;

        auto current = b_closed_list.find(boundary.id());
        while(current->second != goal){
            path_b.push_back(current->second);
            auto it = b_closed_list.find(current->second.pid());
            if(it != b_closed_list.end()){
                current = it;
            } else{
                return {};
            }
        }
        path_b.push_back(goal);

        for(auto rit = path_b.rbegin(); rit != path_b.rend(); ++rit){
            path.push_back(*rit);
        }    

        current = f_closed_list.find(boundary.id());
        while(current->second != start){
            auto it = f_closed_list.find(current->second.pid());
            if(it != f_closed_list.end()){
                current = it;
            } else{
                return {};
            }
            path.push_back(current->second);
        }

        return path;
    }

protected:
    int map_size_;                             // 代价地图的像素数量
    float factor_;                             // 障碍物因子,值越大障碍物的影响越大
    costmap_2d::Costmap2DROS* costmap_ros_;    // 代价地图的ROS封装
    costmap_2d::Costmap2D* costmap_;           // 代价地图缓冲区
};
} // namespace path_planner

#endif
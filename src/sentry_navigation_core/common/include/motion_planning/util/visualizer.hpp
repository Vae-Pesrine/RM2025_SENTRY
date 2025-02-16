#ifndef MOTION_PLANNING_UTIL_VISUALIZER_HPP_
#define MOTION_PLANNING_UTIL_VISUALIZER_HPP_

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include "motion_planning/geometry/point.hpp"
#include "motion_planning/structure/singleton.hpp"

namespace common
{

namespace util
{
class Visualizer
{
public:
    using Point2d = common::geometry::Point2d;
    using Points2d = common::geometry::Points2d;
    using Point3d = common::geometry::Point3d;
    using Points3d = common::geometry::Points3d;
    using Line2d = std::pair<Point2d, Point2d>;
    using Lines2d = std::vector<Line2d>;

public:
    /**
     * @brief 发布路径
     * @param plan 规划路径
     * @param publisher 发布器
     * @param frame_id 参考坐标系
     */ 
    template <typename Points>
    static void publishPlan(const Points& plan, const ros::Publisher& publisher, const std::string& frame_id)
    {
        nav_msgs::Path gui_plan;
        gui_plan.poses.resize(plan.size());
        gui_plan.header.frame_id = frame_id;
        gui_plan.header.stamp = ros::Time::now();
        for(size_t i = 0; i < plan.size(); ++i){
            gui_plan.poses[i].header.stamp = ros::Time::now();
            gui_plan.poses[i].header.frame_id = frame_id;
            gui_plan.poses[i].pose.position.x = plan[i].x();
            gui_plan.poses[i].pose.position.y = plan[i].y();
            gui_plan.poses[i].pose.position.z = 0.0;
            gui_plan.poses[i].pose.orientation.x = 0.0;
            gui_plan.poses[i].pose.orientation.y = 0.0;
            gui_plan.poses[i].pose.orientation.z = 0.0;
            gui_plan.poses[i].pose.orientation.w = 1.0;
        }

        publisher.publish(gui_plan);
    }

    /**
     * @brief 发布扩展区域
     * @param expand 扩展区域
     * @param costmap 代价地图
     * @param publisher 发布器
     * @param frame_id 参考坐标系
     */
    template <typename Points>
    static void publishExpandZone(const Points& expand, const costmap_2d::Costmap2D* costmap,
                                  const ros::Publisher& publisher, const std::string& frame_id)
    {
        nav_msgs::OccupancyGrid grid;
        float resolution = costmap->getResolution();

        // build expand
        grid.header.frame_id = frame_id;
        grid.header.stamp = ros::Time::now();
        grid.info.resolution = resolution;
        grid.info.width = costmap->getSizeInCellsX();
        grid.info.height = costmap->getSizeInCellsY();

        double wx, wy;
        costmap->mapToWorld(0, 0, wx, wy);
        grid.info.origin.position.x = wx - resolution/2;
        grid.info.origin.position.y = wy - resolution/2;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.resize(grid.info.width * grid.info.height);

        for(size_t i = 0; i < grid.data.size(); ++i){
            grid.data[i] = 0;
        }

        auto grid2index = [&](const Point3d& pt){
            return static_cast<int>(pt.x()) + static_cast<int>(grid.info.width * pt.y());
        };

        for(const auto& pt : expand){
            grid.data[grid2index(pt)] = 50;
        }

        publisher.publish(grid);
    }

    /**
     * @brief 发布点
     * @param points 点集合
     * @param publisher 发布器
     * @param frame_id 参考坐标系
     * @param ns 命名空间
     * @param color 点的颜色，默认为红色
     * @param scale 点的大小，默认为 0.2
     * @param type 点的类型，默认为球体（SPHERE）
     */
    template <typename Points>
    static void publishPoints(const Points& points, const ros::Publisher& publisher, const std::string& frame_id,
                              const std::string& ns, std_msgs::ColorRGBA color = RED, double scale = 0.2, int type = SPHERE)
    {
        int cnt = 0;
        visualization_msgs::MarkerArray sphere_array;
        decltype(visualization_msgs::Marker::SPHERE) marker_type;
        if(type == SPHERE){
            marker_type = visualization_msgs::Marker::SPHERE;
        } else if(type == CUBE){
            marker_type = visualization_msgs::Marker::CUBE;
        } else{
            marker_type = visualization_msgs::Marker::SPHERE;
        }

        for (const auto& pt : points)
        {
          visualization_msgs::Marker sphere_mark;
          sphere_mark.header.frame_id = "map";
          sphere_mark.header.stamp = ros::Time::now();
          sphere_mark.ns = ns + "_" + "sphere_mark";
          sphere_mark.id = cnt;
          sphere_mark.type = marker_type;
          sphere_mark.action = visualization_msgs::Marker::ADD;
          sphere_mark.pose.position.x = pt.x();
          sphere_mark.pose.position.y = pt.y();
          sphere_mark.pose.position.z = 0.0;
          sphere_mark.pose.orientation.x = 0.0;
          sphere_mark.pose.orientation.y = 0.0;
          sphere_mark.pose.orientation.z = 0.0;
          sphere_mark.pose.orientation.w = 1.0;
          sphere_mark.lifetime = ros::Duration(1.0);
          sphere_mark.scale.x = scale;
          sphere_mark.scale.y = scale;
          sphere_mark.scale.z = scale;
          sphere_mark.color.r = color.r;
          sphere_mark.color.g = color.g;
          sphere_mark.color.b = color.b;
          sphere_mark.color.a = color.a;
          cnt++;
          sphere_array.markers.push_back(sphere_mark);
        }
        publisher.publish(sphere_array);
        sphere_array.markers.clear();
    }

    /**
     * @brief 发布线段
     * @param lines 线段集合
     * @param publisher 发布器
     * @param frame_id 参考坐标系
     * @param ns 命名空间
     * @param color 线段的颜色，默认为红色
     * @param scale 线段的大小，默认为 0.2
     */
    static void publishLines2d(const Lines2d& lines, const ros::Publisher& publisher, const std::string& frame_id,
                               const std::string& ns, std_msgs::ColorRGBA color = RED, double scale = 0.2);
private:
    /**
     * @brief 初始化颜色
     * @param r 红色分量
     * @param g 绿色分量
     * @param b 蓝色分量
     * @param a 透明度
     * @return 初始化后的颜色
     */
    static std_msgs::ColorRGBA _colorInit(double r, double g, double b, double a);

public:
    static std_msgs::ColorRGBA RED;
    static std_msgs::ColorRGBA DARK_GREEN;
    static std_msgs::ColorRGBA PURPLE;
    enum MARKER_TYPE
    {
        CUBE = 0,    // 立方体
        SPHERE = 1,  // 球体
    };

};
using VisualizerPtr = common::structure::Singleton<Visualizer>;

} // namespace util
} // namespace common

#endif
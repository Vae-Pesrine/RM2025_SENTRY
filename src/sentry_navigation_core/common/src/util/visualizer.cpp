#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <visualization_msgs/MarkerArray.h>

#include "motion_planning/util/visualizer.hpp"

namespace common
{
namespace util
{
std_msgs::ColorRGBA Visualizer::RED = Visualizer::_colorInit(1.0, 0.0, 0.0, 1.0);
std_msgs::ColorRGBA Visualizer::DARK_GREEN = Visualizer::_colorInit(0.43, 0.54, 0.24, 0.5);
std_msgs::ColorRGBA Visualizer::PURPLE = Visualizer::_colorInit(1.0, 0.0, 1.0, 1.0);

/**
 * @brief 发布线段
 * @param lines 线段集合
 * @param publisher 发布器
 * @param frame_id 参考坐标系
 * @param ns 命名空间
 * @param color 线段的颜色，默认为红色
 * @param scale 线段的大小，默认为 0.2
 */
void Visualizer::publishLines2d(const Lines2d& lines, const ros::Publisher& publisher, const std::string& frame_id,
                                const std::string& ns, std_msgs::ColorRGBA color, double scale)
{
    int cnt = 0;
    visualization_msgs::MarkerArray line_array;
    for(const auto& line : lines){
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = ns + "_" + "line_marker";
        line_marker.id = cnt;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point start, end;
        start.x = line.first.x();
        start.y = line.first.y();
        end.x = line.second.x();
        end.y = line.second.y();
        line_marker.points.push_back(start);
        line_marker.points.push_back(end);
        line_marker.lifetime = ros::Duration(0.5);
        line_marker.scale.x = scale;
        line_marker.scale.y = scale;
        line_marker.scale.z = scale;
        line_marker.color.r = color.r;
        line_marker.color.g = color.g;
        line_marker.color.b = color.b;
        line_marker.color.a = color.a;
        cnt++;
        line_array.markers.push_back(line_marker);
    }
    publisher.publish(line_array);
    line_array.markers.clear();
}

/**
 * @brief 初始化颜色
 * @param r 红色分量
 * @param g 绿色分量
 * @param b 蓝色分量
 * @param a 透明度
 * @return 初始化后的颜色
 */
std_msgs::ColorRGBA Visualizer::_colorInit(double r, double g, double b, double a)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

} // namespace util
} // namespace common
#include <cmath>
#include <utility>
#include "motion_planning/math/math_tools.hpp"

namespace common
{
namespace math
{

/**
 * @brief 计算两个向量的叉积
 * @param 
 * end_point_1   *  
 *              *
 * start_point * * *  end_point_2
 * @return 叉积结果
 */
double crossProd(const common::geometry::Vector2d& start_point, const common::geometry::Vector2d& end_point_1,
                 const common::geometry::Vector2d& end_point_2)
{
    return (end_point_1 - start_point).crossProd(end_point_2 - start_point);
}

/**
 * @brief 计算两个向量的内积
 * @param 
 * end_point_1   *  
 *              *
 * start_point * * *  end_point_2
 * @return e内积结果
 */
double innerProd(const common::geometry::Vector2d& start_point, const common::geometry::Vector2d& end_point_1,
                 const common::geometry::Vector2d& end_point_2)
{
    return (end_point_1 - start_point).innerProd(end_point_2 - start_point);
}

/**
 * @brief 计算两个二维向量的叉积
 * @return 叉积结果
 */
double crossProd(const double x0, const double y0, const double x1, const double y1)
{
    return x0 * y1 - y0 * x1;
}

/**
 * @brief 计算两个二维向量的内积
 * @return 内积结果
 */
double innerProd(const double x0, const double y0, const double x1, const double y1)
{
    return x0 * x1 + y0 * y1;
}

/**
 * @brief 确保角度值在 [0, 2*pi] 范围内。
 * @param theta 输入的角度值（单位：弧度）。
 * @return theta_m 模 \(2\pi\) 操作后的角度值。
 */
double mod2pi(double theta)
{   // floor()取整
    return theta - 2.0 * M_PI * floor(theta / M_PI / 2.0);
}

/**
 * @brief 将角度限制在-pi到pi范围内
 * @return 限制后的角度
 */
double pi2pi(double theta)
{
    while(theta > M_PI) theta -= 2.0 * M_PI;
    while(theta < -M_PI) theta += 2.0 * M_PI;
    return theta;
}

/**
 * @brief 计算线段与以原点为中心的圆的交点。
 * 
 * 该函数计算线段 P1P2与以原点为中心、半径为 r 的圆的交点。
 * 如果线段与圆相交，返回交点坐标；否则返回空列表。
 * 
 * @param p1 线段的起点坐标 (x1, y1)。
 * @param p2 线段的终点坐标 (x2, y2)。
 * @param r 圆的半径。
 * @return std::vector<std::pair<double, double>> 交点坐标列表（可能为空）。
 */
std::vector<std::pair<double, double>> circleSegmentIntersection(const std::pair<double, double>& p1,
                                                                 const std::pair<double, double>& p2, double r)
{
    std::vector<std::pair<double, double>> i_points;

    double x1 = p1.first;
    double x2 = p2.first;
    double y1 = p1.second;
    double y2 = p2.second;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr2 = dx * dx + dy * dy;
    double D = x1 * y2 - x2 * y1;

    double d1 = x1 * x1 + y1 * y1;
    double d2 = x2 * x2 + y2 * y2;
    double dd = d2 - d1;

    double delta = std::sqrt(r * r * dr2 - D * D);
    if(delta >=0){
        if(delta == 0){
            //一个交点
            i_points.emplace_back(D * dy / dr2, -D * dx / dr2);
        }else{
            //两个交点
            i_points.emplace_back((D * dy + std::copysign(1.0, dd) * dx * delta) / dr2,
                                  (-D * dx + std::copysign(1.0, dd) * dy * delta) / dr2);
            i_points.emplace_back((D * dy - std::copysign(1.0, dd) * dx * delta) / dr2,
                                  (-D * dx - std::copysign(1.0, dd) * dy * delta) / dr2);
        }
    }

    return i_points;
}

} // namespace math
} // namespace common
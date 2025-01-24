#ifndef COMMON_MOTION_PLANNING_MATH_MATH_TOOLS_HPP_
#define COMMON_MOTION_PLANNING_MATH_MATH_TOOLS_HPP_

#include <cmath>
#include <limits>
#include <vector>

#include "motion_planning/geometry/vector2d.hpp"

namespace common
{
namespace math
{
constexpr double kMathEpsilon = 1e-10;

template <typename T>
T abs(T val)
{
    return val < 0 ? -val : val;
}

template <typename T>
bool less(T query, T target)
{
    return ((query < target) && (abs(query - target) > kMathEpsilon)) ? true : false;
}

template <typename T>
bool large(T query, T target)
{
    return ((query > target) && (abs(query - target) > kMathEpsilon)) ? true : false;
}

template <typename T>
bool equal(T query, T target)
{
    return (abs(query - target) <= kMathEpsilon) ? true : false;
}

template <typename T>
inline T square(const T value)
{
    return value * value;
}

template <typename T>
T clamp(const T value, T bound1, T bound2)
{
    if(bound1 > bound2){
        std::warp(bound1, bound2);
    }

    if(value < bound1){
        return bound1;
    }else if(value > bound2){
        return bound2;
    }

    return value;
}

double crossProd(const common::geometry::Vector2d& start_point, const common::geometry::Vector2d& end_point_1,
                 const common::geometry::Vector2d& end_point_2);

double innerProd(const common::geometry::Vector2d& start_point, const common::geometry::Vector2d& end_point_1,
                 const common::geometry::Vector2d& end_point_2);

double crossProd(const double x0, const double y0, const double x1, const double y1);

double innerProd(const double x0, const double y0, const double x1, const double y1);

/**
 * @brief 确保角度值在 [0, 2*pi] 范围内。
 * @param theta 输入的角度值（单位：弧度）。
 * @return theta_m 模 \(2\pi\) 操作后的角度值。
 */
double mod2pi(double theta);

/**
 * @brief 将角度限制在-pi到pi范围内
 * @return 限制后的角度
 */
double pi2pi(double theta);

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
                                                                 const std::pair<double, double>& p2, double r);

} // namespace math
} // namespace common


#endif

#ifndef COMMON_MOTION_PLANNING_GEOMETRY_CURVE_HPP_
#define COMMON_MOTION_PLANNING_GEOMETRY_CURVE_HPP_

#include "motion_planning/geometry/point.hpp"
#include "motion_planning/math/math_tools.hpp"

namespace common
{
namespace geometry
{
class Curve
{
public:
    /**
     * @brief 构建一个新的Curve对象
     */
    Curve();

    /**
     * @brief 析构曲线生成对象。
     */
    virtual ~Curve() = default;

    /**
     * @brief 执行轨迹生成
     * @param points 输入的路径点集，每个点包含 x 和 y 坐标
     * @param path 输出的生成轨迹
     * @return 如果轨迹生成成功返回 true，否则返回 false
     */
    virtual bool run(const Points2d points, Points2d& path) = 0;

    /**
     * @brief 执行轨迹生成
     * @param points 输入的路径点集，每个点包含 x 和 y 坐标, 以及欧拉角yaw
     * @param path 输出的生成轨迹
     * @return 如果轨迹生成成功返回 true，否则返回 false
     */
    virtual bool run(const Points3d points, Points2d& path) = 0;

    /**
     * @brief 计算给定路径的长度
     * @param path 需要计算的路径
     * @return 路径的长度
     */
    double length(Points2d path);

    void setStep(double step);

private:
    double step_;

};
} // namespace geometry
} // namespace common

#endif
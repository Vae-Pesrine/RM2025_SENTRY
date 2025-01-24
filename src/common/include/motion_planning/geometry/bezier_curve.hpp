#ifndef COMMON_MOTION_PLANNING_GEOMETRY_BEZIER_CURVE_HPP_
#define COMMON_MOTION_PLANNING_GEOMETRY_BEZIER_CURVE_HPP_

#include "motion_planning/geometry/curve.hpp"

namespace common
{
namespace geometry
{
class BezierCurve: public Curve
{
public:
    /**
     * @brief 构造一个新的贝塞尔曲线生成对象。
     * @param step 仿真或插值的步长大小（默认值：0.1）控制曲线生成时的精度,步长越小生成的曲线越平滑,但计算量会增加
     * @param offset 控制点的偏移量（默认值：3.0） 控制贝塞尔曲线的形状
     */
    BezierCurve();
    BezierCurve(double step, double offset);
    
    /**
     * @brief 析构贝塞尔曲线生成对象。
     */
    ~BezierCurve();

    /**
     * @brief 计算贝塞尔曲线上的点。
     * @param t 缩放因子（通常在 [0, 1] 范围内）
     * @param control_pts 控制点集合
     * @return point 返回贝塞尔曲线上对应参数 t 的点
     */
    Point2d bezier(double t, Points2d control_pts);

    /* @brief 启发式计算控制点。
     * @param start 初始姿态 (x, y, yaw)
     * @param goal  目标姿态 (x, y, yaw)
     * @return control_pts 控制点集合
     */
    Points2d getControlPoints(Point3d start, Point3d goal);
    
    /**
     * @brief 生成路径。
     * @param start 初始姿态 (x, y, yaw)
     * @param goal  目标姿态 (x, y, yaw)
     * @return path 生成的平滑轨迹点
     */
    Points2d generation(Point3d start, Point3d goal);
    /**
     * @brief 执行轨迹生成。
     * @param points 输入的路径点集（二维点，包含 x 和 y 坐标）。
     * @param path 输出的生成轨迹。
     * @return 如果轨迹生成成功返回 true，否则返回 false。
     */
    bool run(const Points2d points, Points2d& path);

    /**
     * @brief 执行轨迹生成。
     * @param points 输入的路径点集（三维点，包含 x、y 和朝向 theta）。
     * @param path 输出的生成轨迹（二维点集，仅包含 x 和 y 坐标）。
     * @return 如果轨迹生成成功返回 true，否则返回 false。
     */
    bool run(const Points3d points, Points2d& path);

    /**
     * @brief 设置控制点的偏移量。
     * @param offset 控制点的偏移量
     */
    void setOffset(double offset);
    
    /**
     * @brief 设置贝塞尔曲线生成的步长。 步长用于控制生成轨迹的精度。步长越小，生成的轨迹越平滑，但计算量也会增加。
     * @param step 生成轨迹的步长，必须大于 0。
     */
    void setStep(double step);

private:

    /**
     * @brief 计算组合数 \( C(n, r) \)。
     * 
     * 使用数学公式计算组合数，避免递归带来的重复计算问题。
     * 组合数公式：\( C(n, r) = \frac{n!}{r! (n - r)!} \)
     * 为了防止溢出和提高效率，采用逐步计算的方式。
     * 
     * @param n 总元素数量。
     * @param r 选择的元素数量。
     * @return int 计算得到的组合数。
     */
    int _comb(int n, int r);

protected:
    double step_;   //步长 
    double offset_; //控制点的偏移量

};
} // namespace geometry
} // namespace common


#endif
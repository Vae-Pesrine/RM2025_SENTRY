#include <cassert>
#include "motion_planning/util/log.hpp"
#include "motion_planning/geometry/bezier_curve.hpp"

namespace common
{
namespace geometry
{
/**
 * @brief 构造一个新的贝塞尔曲线生成对象。
 * @param step 仿真或插值的步长大小（默认值：0.1）控制曲线生成时的精度,步长越小生成的曲线越平滑,但计算量会增加
 * @param offset 控制点的偏移量（默认值：3.0） 控制贝塞尔曲线的形状
 */
BezierCurve::BezierCurve(double step, double offset): step_(step), offset_(offset)
{
}
BezierCurve::BezierCurve() : step_(0.1), offset_(3)
{
}

/**
 * @brief 析构贝塞尔曲线生成对象。
 */
BezierCurve::~BezierCurve()
{
}

/**
 * @brief 计算贝塞尔曲线上的点。
 * @param t 缩放因子（通常在 [0, 1] 范围内）
 * @param control_pts 控制点集合
 * @return point 返回贝塞尔曲线上对应参数 t 的点
 */
Point2d BezierCurve::bezier(double t, Points2d control_pts)
{
    size_t n = control_pts.size() - 1;
    double pt_x = 0, pt_y = 0;
    for(size_t i = 0; i < n + 1; ++i){
        pt_x += _comb(n, i) * std::pow(t, i) * std::pow(1-t, n-i) * control_pts[i].x();
        pt_y += _comb(n, i) * std::pow(t, i) * std::pow(1-t, n-i) * control_pts[i].y();
    }

    return {pt_x, pt_y};
}

/**
 * @brief 启发式计算控制点。
 * @param start 初始姿态 (x, y, yaw)
 * @param goal  目标姿态 (x, y, yaw)
 * @return control_pts 控制点集合
 */
Points2d BezierCurve::getControlPoints(Point3d start, Point3d goal)
{
    double sx = start.x(), sy = start.y(), syaw = start.theta();
    double gx = goal.x(),  gy = goal.y(),  gyaw = goal.theta();

    double d = std::hypot(sx - gx, sy - gy) / offset_;


    int n = (static_cast<int>(offset_)% 2 == 0) ? offset_ / 2 : (offset_ - 1) / 2;
    Points2d control_pts;
    for(size_t i = 0; i < n; ++i){
        control_pts.emplace_back(sx + i * d * cos(syaw), sy + i * d * sin(syaw));
    }
    for(size_t i = n-1; i >= 0; --i){
        control_pts.emplace_back(gx - i * d * cos(gyaw), gy - i * d * sin(gyaw));
    }

    return control_pts;
}

/**
 * @brief 生成路径。
 * @param start 初始姿态 (x, y, yaw)
 * @param goal  目标姿态 (x, y, yaw)
 * @return path 生成的平滑轨迹点
 */
Points2d BezierCurve::generation(Point3d start, Point3d goal)
{
    double sx = start.x(), sy = start.y(), syaw = start.theta();
    double gx = goal.x(),  gy = goal.y(),  gyaw = goal.theta();

    int n_points = static_cast<int>(std::hypot(sx - gx, sy - gy) / step_);
    Points2d control_pts = getControlPoints(start, goal);

    Points2d points;
    for(size_t i = 0; i < n_points; ++i){
        // 参数 t 的值从 0 到 1
        double t = (double)(i) / (double)(n_points - 1);
        points.push_back(bezier(t, control_pts));
    }

    return points;
}

/**
 * @brief 执行轨迹生成。
 * @param points 输入的路径点集（二维点，包含 x 和 y 坐标）。
 * @param path 输出的生成轨迹。
 * @return 如果轨迹生成成功返回 true，否则返回 false。
 */
bool BezierCurve::run(const Points2d points, Points2d& path)
{
    if(points.size() < 4){
        return false;
    } else{
        Points3d poses;
        poses.emplace_back(points.begin()->x(), points.begin()->y(), 0);
        for(size_t i = 0; i < points.size(); ++i){
            double theta1 = std::atan2(points[i].y() - points[i-1].y(), points[i].x() - points[i-1].x());
            double theta2 = std::atan2(points[i+1].y() - points[i].y(), points[i+1].x() - points[i].x());
            poses.emplace_back(points[i].x(), points[i].y(), (theta1 + theta2) / 2);
        }
        poses.emplace_back(points.back().x(), points.back().y(), 0);
    
        return run(poses, path);
    }
}

/**
 * @brief 执行轨迹生成。
 * @param points 输入的路径点集（三维点，包含 x、y 和朝向 theta）。
 * @param path 输出的生成轨迹（二维点集，仅包含 x 和 y 坐标）。
 * @return 如果轨迹生成成功返回 true，否则返回 false。
 */
bool BezierCurve::run(const Points3d points, Points2d& path)
{
    if(points.size() < 4){
        return false;
    } else{
        path.clear();
        for(size_t i = 0; i < points.size(); ++i){
            // 生成每一段的贝塞尔曲线
            Points2d path_i = generation(points[i], points[i+1]);
            // 将生成的路径点追加到总路径中
            path.insert(path.end(), path_i.begin(), path_i.end());
        }

        return !path.empty();
    }
}

/**
 * @brief 设置控制点的偏移量。
 * @param offset 控制点的偏移量
 */
void BezierCurve::setOffset(double offset)
{
    CHECK_GT(offset, 0.0);
    offset_ = offset;
}

/**
 * @brief 设置贝塞尔曲线生成的步长。 步长用于控制生成轨迹的精度。步长越小，生成的轨迹越平滑，但计算量也会增加。
 * @param step 生成轨迹的步长，必须大于 0。
 */
void BezierCurve::setStep(double step)
{
    CHECK_GT(step, 0.0);
    step_ = step;
}

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
int BezierCurve::_comb(int n, int r)
{
    if (r > n) return 0;
    if (r == 0 || r == n) return 1;

    double result = 1.0;
    for (int i = 1; i <= r; ++i) {
        result *= (n - i + 1);
        result /= i;
    }
    return static_cast<int>(result);
}

} // namespace geometry
} // namespace common 
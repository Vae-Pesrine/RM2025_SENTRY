#include <cassert>
#include <Eigen/Dense>
#include "motion_planning/geometry/bspline_curve.hpp"

namespace common
{
namespace geometry
{
/**
 * @brief 构造一个新的 B 样条曲线生成对象。
 * 
 * @param step 仿真或插值的步长大小（默认值：0.01）。
 * @param order 曲线的阶数（默认值：3，即三次 B 样条曲线）。
 * @param param_mode 参数化模式（默认值：PARAM_MODE::CHORDLENGTH，弦长参数化）。
 * @param spline_mode B 样条曲线生成模式（默认值：SPLINE_MODE::INTERPOLATION，插值模式）。
 */
BsplineCurve::BsplineCurve(double step, int order, int param_mode, int spline_mode):
    step_(step), order_(order), param_mode_(param_mode), spline_mode_(spline_mode)
{
}
BsplineCurve::BsplineCurve()
    : step_(0.01), order_(3), param_mode_(PARAM_MODE::CENTRIPETAL), spline_mode_(SPLINE_MODE::INTERPOLATION)
{
}

/**
 * @brief 析构 B 样条曲线生成对象。
 */
BsplineCurve::~BsplineCurve()
{
}

/**
 * @brief 使用德布尔-考克斯递推式计算B样条基函数
 * @param i 索引
 * @param k 曲线的阶数 
 * @param t 参数 
 * @param knot 节点
 * @return Nik_t k阶样条曲线是关于t的k-1次函数
 */
double BsplineCurve::baseFunction(int i, int k, double t, std::vector<double> knot)
{
    double Nik_t = 0;

    //一阶
    if(k == 0){
        Nik_t = ((t >= knot[i]) && (t < knot[i+1])) ? 1.0 : 0.0;
    }
    //二阶
    else{
        double length1 = double(knot[i+k]) - knot[i];
        double length2 = double(knot[i+k+1]) - knot[i+1];

        if(length1 == 0 && length2 == 0)
            Nik_t = 0;
        else if(length1 == 0)
            Nik_t = (knot[i + k + 1] - t) / length2 * baseFunction(i + 1, k - 1, t, knot);
        else if (length2 == 0)
            Nik_t = (t - knot[i]) / length1 * baseFunction(i, k - 1, t, knot);
        else
            Nik_t = (t - knot[i]) / length1 * baseFunction(i, k - 1, t, knot) +
                    (knot[i + k + 1] - t) / length2 * baseFunction(i + 1, k - 1, t, knot);
    } 

    return Nik_t;
}


/**
 * @brief 计算路径点的参数值，支持均匀参数化、弦长参数化和离心参数化。
 * 
 * 参数化方法决定了路径点在参数空间中的分布方式，影响 B 样条曲线的形状。
 * 支持的参数化方法包括：
 * - 均匀参数化（Uniform Spaced）：参数值均匀分布, 适用于点分布较为均匀的情况
 * - 弦长参数化（Chord Length）：参数值基于点之间的欧几里得距离, 适用于点分布不均匀的情况
 * - 离心参数化（Centripetal）：参数值基于点之间的欧几里得距离的平方根, 适用于曲线较为复杂的情况
 * 
 * @param points 输入的路径点集。
 * @return std::vector<double> 计算得到的参数值。
 */
std::vector<double> BsplineCurve::paramSelection(const Points2d points)
{
    size_t n = points.size();
    std::vector<double> parameters(n);

    if(param_mode_ == PARAM_MODE::UNIFORMSPACED){
        for(size_t i = 0; i < n; ++i){
            parameters[i] = (double)(i) / (double)(n-1);
        }
    }else{
        parameters[0] = 0.0;
        std::vector<double> s(n-1);
        
        double d_cumsum = 0.0;
        for(size_t i = 0; i < n - 1; ++i){
            double d;
            if(param_mode_ == PARAM_MODE::CHORDLENGTH){
                d = std::hypot(points[i].x() - points[i + 1].x(), points[i].y() - points[i + 1].y());
            }else{
                d = std::pow(std::hypot(points[i].x() - points[i + 1].x(), points[i].y() - points[i + 1].y()), 0.5);
            }

            d_cumsum += d;
            s[i] = d_cumsum;
        }

        for(size_t i = 0; i < n; ++i){
            parameters[i] = s[i - 1] / s[n - 2];
        }
    }

    return parameters;
}


/**
 * @brief 生成 B 样条曲线的节点向量。
 * 
 * 节点向量的生成基于给定的参数值和数据点的数量。
 * 节点向量的长度为 m = n + order + 1，其中 n 是数据点的数量, order 是 B 样条曲线的阶数。
 * 节点向量的生成方法通常采用均匀分布或基于参数值的加权平均。
 * 
 * @param param 给定数据点的参数值。
 * @param n 数据点的数量。
 * @return std::vector<double> 生成的节点向量。
 */
std::vector<double> BsplineCurve::knotGeneration(const std::vector<double> param, int n)
{
    int m = n + order_ + 1;
    std::vector<double> knot(m);

    for(size_t i = 0; i < n; ++i){
        knot[i] = 0.0;
    }
    for (size_t i = n; i < m; i++){
        knot[i] = 1.0;
    }
    for(size_t i = order_ + 1; i < n; ++i){
        for(size_t j = i - order_; j < i; ++j){
            knot[i] += param[j];
        }
        knot[i] /= order_;
    }
    
    return knot;
}

/**
 * @brief 给定一组数据点和曲线的阶数，找到一组控制点，使得 B 样条曲线通过所有数据点。
 * 
 * 该函数实现 B 样条曲线的插值算法。具体步骤如下：
 * 1. 构建基函数矩阵 N ，其中 N{ij} = N{j,k}(t_i) \)，表示第 j 个控制点对应的基函数在参数 t_i 处的值。
 * 2. 构建数据点矩阵 D ，包含所有数据点的坐标。
 * 3. 通过求解线性方程组  N * C = D ，得到控制点矩阵  C 。
 * 4. 将控制点矩阵转换为控制点集合。
 * 
 * @param points 输入的数据点集合。
 * @param param 数据点对应的参数值。
 * @param knot 节点向量。
 * @return Points2d 计算得到的控制点集合。
 */
Points2d BsplineCurve::interpolation(const Points2d points, const std::vector<double> param, const std::vector<double> knot)
{
    size_t n = points.size();
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero();
    Eigen::MatrixXd D(n, 2);

    for(size_t i = 0; i < n; ++i){
        for(size_t j = 0; j < n ; ++j){
            N(i, j) = baseFunction(j, order_, param[i], knot);
        }
    }
    N(n - 1, n - 1) = 1;

    for(size_t i = 0; i < n; ++i){
        D(i, 0) = points[i].x();
        D(i, 1) = points[i].y();
    }
    Eigen::MatrixXd C = N.inverse() * D;

    Points2d control_points(n);
    for(size_t i = 0; i < n; ++i){
        control_points[i] = { C(i, 0), C(i, 1)};
    }

    return control_points;
}

/**
 * @brief 给定一组数据点和曲线的阶数，找到一组控制点，使得 B 样条曲线在最小二乘意义上逼近所有数据点。
 * 
 * 该函数实现 B 样条曲线的最小二乘拟合算法。具体步骤如下：
 * 1. 构建数据点矩阵 D ，包含所有数据点的坐标。
 * 2. 构建基函数矩阵 N ，其中 N{ij} = N{j,k}(t_i) ，表示第 j 个控制点对应的基函数在参数 t_i 处的值。
 * 3. 构建简化后的基函数矩阵 N_ 和数据点矩阵 qk ，用于求解中间控制点。
 * 4. 通过求解线性方程组 N_ * P = qk ，得到中间控制点矩阵 P 。
 * 5. 将控制点矩阵转换为控制点集合。
 * 
 * @param points 输入的数据点集合。
 * @param param 数据点对应的参数值。
 * @param knot 节点向量。
 * @return Points2d 计算得到的控制点集合。
 */
Points2d BsplineCurve::approximation(const Points2d points, const std::vector<double> param, const std::vector<double> knot)
{
    size_t n = points.size();
    Eigen::MatrixXd D(n, 2);
    for(size_t i = 0; i < n; ++i){
        D(i, 0) = points[i].x();
        D(i, 1) = points[i].y();
    }

    size_t h = n - 1;
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(n, h);
    for(size_t i = 0; i < n; ++i){
        for (size_t j = 0; j < h; ++j){
            N(i, j) = baseFunction(j, order_, param[i], knot);
        }
    }

    Eigen::MatrixXd N_ = Eigen::MatrixXd::Zero(n-2, h-2);
    for(size_t i = 1; i , n - 1; ++i){
        for(size_t j = 1; j < h - 1; ++j){
            N_(i-1, j-1) = N(i, j);
        }
    }   

    Eigen::MatrixXd qk = Eigen::MatrixXd::Zero(n-2, 2);
    for(size_t i = 1; i < n - 1; ++i){
        qk(i-1, 0) = D(i, 0) - N(i, 0) * D(0, 0) - N(i, h-1) * D(n-1, 0);
        qk(i-1, 1) = D(i, 1) - N(i, 0) * D(0, 1) - N(i, h-1) * D(n-1, 1);
    }

    Eigen::MatrixXd Q = N_.transpose() * qk;
    Eigen::MatrixXd P = (N_.transpose() * N_).inverse() * Q;

    Points2d control_points(h);
    control_points[o] = { D(0, 0), D(0, 1) };
    control_points[h - 1] = { D(n-1, 0), D(n-1, 1) };
    for(size_t i = 1; i < h - 1; ++i){
        control_points[i] = { P(i-1, 0), P(i-1, 1) };
    }

    return control_points;
}

/**
 * @brief 生成 B 样条曲线的平滑轨迹点。
 * 
 * 该函数根据给定的曲线阶数、节点向量和控制点，计算基函数矩阵和控制点矩阵的乘积，
 * 生成平滑的轨迹点。
 * 
 * @param k 曲线的阶数。
 * @param knot 节点向量。
 * @param control_pts 控制点集合。
 * @return Points2d 生成的平滑轨迹点集合。
 */
Points2d BsplineCurve::generation(int k, const std::vector<double> knot, Points2d control_pts)
{
    size_t n = static_cast<int>(1.0 / step_);
    std::vector<double> t(n);
    for(size_t i = 0; i < n; ++i){
        t[i] = (double)(i) / (double)(n - 1);
    }

    Eigen::MatrixXd N(n, control_pts.size());
    for(size_t i = 0; i < n; ++i){
        for(size_t j = 0; j < control_pts.size(); ++j){
            N(i, j) = baseFunction(j, order_, t[i], knot);
        }
    }

    N(n-1, control_pts.size()-1) = 1.0;

    Eigen::MatrixXd C(control_pts.size(), 2);
    for(size_t i = 0; i < control_pts.size(); ++i){
        C(i, 0) = control_pts[i].x();
        C(i, 1) = control_pts[i].y();
    }

    Eigen::MatrixXd P = N * C;
    Points2d points(n);
    for(size_t i = 0; i < n; ++i){
        points[i] = { P(i, 0), P(i, 1) };
    }

    return points;
}

/**
 * @brief 执行轨迹生成。
 * @param points 输入的路径点集合, 包含x和y坐标
 * @param path 输出的生成轨迹。
 * @return 如果轨迹生成成功返回 true，否则返回 false。
 */
bool BsplineCurve::run(const Points2d points, Points2d& path)
{
    if(points.size() < 4){
        return false;
    } else{
        Points2d control_pts;
        std::vector<double> params = paramSelection(points);
        std::vector<double> knot = knotGeneration(params, points.size());
        if(spline_mode_ == SPLINE_MODE::INTERPOLATION){
            control_pts = interpolation(points, params, knot);  // 插值方式
        } else if(spline_mode_ == SPLINE_MODE::APPROXIMATION){
            control_pts = approximation(points, params, knot);
            params = paramSelection(points);
            knot = knotGeneration(params, control_pts.size());
        }else{
            return false;
        }

        path = generation(order_, knot, control_pts);

        return !path.empty();
    }
}

/**
 * @brief 执行轨迹生成。
 * @param points 输入的路径点集合, 包含x和y坐标, 以及yaw的theta
 * @param path 输出的生成轨迹。
 * @return 如果轨迹生成成功返回 true，否则返回 false。
 */
bool BsplineCurve::run(const Points3d points, Points2d& path)
{
    Points2d points_pair;
    for(const auto& p : points){
        points_pair.emplace_back(p.x(), p.y());
    }

    return run(points_pair, path);
}

void BsplineCurve::setSplineOrder(int order)
{
    assert(order > 0);
    order = order;
}

void BsplineCurve::setParamMode(int param_mode)
{
    assert((param_mode == PARAM_MODE::CENTRIPETAL) || (param_mode == PARAM_MODE::CHORDLENGTH) ||
           (param_mode == PARAM_MODE::UNIFORMSPACED));
    param_mode_ = param_mode;
}

void BsplineCurve::setSplineMode(int spline_mode)
{
    assert((spline_mode == SPLINE_MODE::APPROXIMATION) || 
           (spline_mode == SPLINE_MODE::INTERPOLATION));
    spline_mode_ = spline_mode;
}

} // namespace geometry
} // namespace common
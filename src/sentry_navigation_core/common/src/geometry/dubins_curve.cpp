#include <Eigen/Dense>
#include <climits>
#include <cassert>
#include <iostream>

#include "motion_planning/math/math_tools.hpp"
#include "motion_planning/geometry/dubins_curve.hpp"

namespace common
{
namespace geometry
{
namespace
{
#define UNPACK_DUBINS_INPUTS(alpha, beta)                                                                              \
  double sin_a = sin(alpha);                                                                                           \
  double sin_b = sin(beta);                                                                                            \
  double cos_a = cos(alpha);                                                                                           \
  double cos_b = cos(beta);                                                                                            \
  double cos_a_b = cos(alpha - beta);
}  // namespace


enum
{
    DUBINS_NONE = -1,
    DUBINS_L = 0,
    DUBINS_S = 1,
    DUBINS_R = 2,
};

DubinsCurve::DubinsCurve(double step, double max_curv): step_(step), max_curv_(max_curv)
{
}

DubinsCurve::DubinsCurve(): step_(0.1), max_curv_(0.25)
{
}

DubinsCurve::~DubinsCurve()
{
}

/**
 * @brief Left-Straight-Left（LSL）模式的 Dubins 曲线生成。
 * 
 * 该函数计算从初始姿态 (0, 0, alpha) 到目标姿态 (dist, 0, beta) 的 LSL 模式路径。
 * LSL 模式表示路径由左转、直行和左转三个部分组成。
 * 
 * @param alpha 初始姿态的朝向角（单位：弧度）。
 * @param beta 目标姿态的朝向角（单位：弧度）。
 * @param dist 目标姿态的 x 坐标（目标点与原点的距离）。
 * @param length 返回的路径段长度（t, s, p）。
 * @param mode 返回的运动模式。
 */
void DubinsCurve::LSL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  UNPACK_DUBINS_INPUTS(alpha, beta);
  double p_lsl = 2 + std::pow(dist, 2) - 2 * cos_a_b + 2 * dist * (sin_a - sin_b);

  if (p_lsl < 0){
    length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
    mode = { DUBINS_L, DUBINS_S, DUBINS_L };
  } else{
    p_lsl = sqrt(p_lsl);
    double t_lsl = common::math::mod2pi(-alpha + atan2(cos_b - cos_a, dist + sin_a - sin_b));
    double q_lsl = common::math::mod2pi(beta - atan2(cos_b - cos_a, dist + sin_a - sin_b));
    length = { t_lsl, p_lsl, q_lsl };
    mode = { DUBINS_L, DUBINS_S, DUBINS_L };
  }
}

/**
 * @brief Right-Straight-Right（RSR）模式的 Dubins 曲线生成。
 * 
 * 该函数计算从初始姿态 (0, 0, alpha) 到目标姿态 (dist, 0, beta) 的 RSR 模式路径。
 * RSR 模式表示路径由右转、直行和右转三个部分组成。
 * 
 * @param alpha 初始姿态的朝向角（单位：弧度）。
 * @param beta 目标姿态的朝向角（单位：弧度）。
 * @param dist 目标姿态的 x 坐标（目标点与原点的距离）。
 * @param length 返回的路径段长度（t, s, p）。
 * @param mode 返回的运动模式。
 */
void DubinsCurve::RSR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  UNPACK_DUBINS_INPUTS(alpha, beta);
  double p_rsr = 2 + std::pow(dist, 2) - 2 * cos_a_b + 2 * dist * (sin_b - sin_a);
  if (p_rsr < 0){
    length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
    mode = { DUBINS_R, DUBINS_S, DUBINS_R };
  } else{
    p_rsr = sqrt(p_rsr);
    double t_rsr = common::math::mod2pi(alpha - atan2(cos_a - cos_b, dist - sin_a + sin_b));
    double q_rsr = common::math::mod2pi(-beta + atan2(cos_a - cos_b, dist - sin_a + sin_b));
    length = { t_rsr, p_rsr, q_rsr };
    mode = { DUBINS_R, DUBINS_S, DUBINS_R };
  }
}

/**
 * @brief Left-Straight-Right（LSR）模式的 Dubins 曲线生成。
 * 
 * 该函数计算从初始姿态 (0, 0, alpha) 到目标姿态 (dist, 0, beta) 的 LSR 模式路径。
 * LSR 模式表示路径由左转、直行和右转三个部分组成。
 * 
 * @param alpha 初始姿态的朝向角（单位：弧度）。
 * @param beta 目标姿态的朝向角（单位：弧度）。
 * @param dist 目标姿态的 x 坐标（目标点与原点的距离）。
 * @param length 返回的路径段长度（t, s, p）。
 * @param mode 返回的运动模式。
 */
void DubinsCurve::LSR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  UNPACK_DUBINS_INPUTS(alpha, beta);
  double p_lsr = -2 + std::pow(dist, 2) + 2 * cos_a_b + 2 * dist * (sin_a + sin_b);

  if (p_lsr < 0){
    length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
    mode = { DUBINS_L, DUBINS_S, DUBINS_R };
  } else{
    p_lsr = sqrt(p_lsr);
    double t_lsr = common::math::mod2pi(-alpha + atan2(-cos_a - cos_b, dist + sin_a + sin_b) - atan2(-2.0, p_lsr));
    double q_lsr = common::math::mod2pi(-beta + atan2(-cos_a - cos_b, dist + sin_a + sin_b) - atan2(-2.0, p_lsr));
    length = { t_lsr, p_lsr, q_lsr };
    mode = { DUBINS_L, DUBINS_S, DUBINS_R };
  }
}

/**
 * @brief Right-Straight-Left（RSL）模式的 Dubins 曲线生成。
 * 
 * 该函数计算从初始姿态 (0, 0, alpha) 到目标姿态 (dist, 0, beta) 的 RSL 模式路径。
 * RSL 模式表示路径由右转、直行和左转三个部分组成。
 * 
 * @param alpha 初始姿态的朝向角（单位：弧度）。
 * @param beta 目标姿态的朝向角（单位：弧度）。
 * @param dist 目标姿态的 x 坐标（目标点与原点的距离）。
 * @param length 返回的路径段长度（t, s, p）。
 * @param mode 返回的运动模式。
 */
void DubinsCurve::RSL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  UNPACK_DUBINS_INPUTS(alpha, beta);
  double p_rsl = -2 + std::pow(dist, 2) + 2 * cos_a_b - 2 * dist * (sin_a + sin_b);

  if (p_rsl < 0){
    length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
    mode = { DUBINS_R, DUBINS_S, DUBINS_L };
  } else{
    p_rsl = sqrt(p_rsl);
    double t_rsl = common::math::mod2pi(alpha - atan2(cos_a + cos_b, dist - sin_a - sin_b) + atan2(2.0, p_rsl));
    double q_rsl = common::math::mod2pi(beta - atan2(cos_a + cos_b, dist - sin_a - sin_b) + atan2(2.0, p_rsl));
    length = { t_rsl, p_rsl, q_rsl };
    mode = { DUBINS_R, DUBINS_S, DUBINS_L };
  }
}

/**
 * @brief Right-Left-Right（RLR）模式的 Dubins 曲线生成。
 * 
 * 该函数计算从初始姿态 (0, 0, alpha) 到目标姿态 (dist, 0, beta) 的 RLR 模式路径。
 * RLR 模式表示路径由右转、左转和右转三个部分组成。
 * 
 * @param alpha 初始姿态的朝向角（单位：弧度）。
 * @param beta 目标姿态的朝向角（单位：弧度）。
 * @param dist 目标姿态的 x 坐标（目标点与原点的距离）。
 * @param length 返回的路径段长度（t, s, p）。
 * @param mode 返回的运动模式。
 */
void DubinsCurve::RLR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  UNPACK_DUBINS_INPUTS(alpha, beta);
  double p_rlr = (6.0 - std::pow(dist, 2) + 2.0 * cos_a_b + 2.0 * dist * (sin_a - sin_b)) / 8.0;

  if (fabs(p_rlr) > 1.0){
    length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
    mode = { DUBINS_R, DUBINS_L, DUBINS_R };
  } else{
    p_rlr = common::math::mod2pi(2 * M_PI - acos(p_rlr));
    double t_rlr = common::math::mod2pi(alpha - atan2(cos_a - cos_b, dist - sin_a + sin_b) + p_rlr / 2.0);
    double q_rlr = common::math::mod2pi(alpha - beta - t_rlr + p_rlr);
    length = { t_rlr, p_rlr, q_rlr };
    mode = { DUBINS_R, DUBINS_L, DUBINS_R };
  }
}

/**
 * @brief Left-Right-Left（LRL）模式的 Dubins 曲线生成。
 * 
 * 该函数计算从初始姿态 (0, 0, alpha) 到目标姿态 (dist, 0, beta) 的 LRL 模式路径。
 * LRL 模式表示路径由左转、右转和左转三个部分组成。
 * 
 * @param alpha 初始姿态的朝向角（单位：弧度）。
 * @param beta 目标姿态的朝向角（单位：弧度）。
 * @param dist 目标姿态的 x 坐标（目标点与原点的距离）。
 * @param length 返回的路径段长度（t, s, p）。
 * @param mode 返回的运动模式。
 */
void DubinsCurve::LRL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  UNPACK_DUBINS_INPUTS(alpha, beta);
  double p_lrl = (6.0 - std::pow(dist, 2) + 2.0 * cos_a_b + 2.0 * dist * (sin_a - sin_b)) / 8.0;

  if (fabs(p_lrl) > 1.0){
    length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
    mode = { DUBINS_L, DUBINS_R, DUBINS_L };
  } else{
    p_lrl = common::math::mod2pi(2 * M_PI - acos(p_lrl));
    double t_lrl = common::math::mod2pi(-alpha + atan2(-cos_a + cos_b, dist + sin_a - sin_b) + p_lrl / 2.0);
    double q_lrl = common::math::mod2pi(beta - alpha - t_lrl + p_lrl);
    length = { t_lrl, p_lrl, q_lrl };
    mode = { DUBINS_L, DUBINS_R, DUBINS_L };
  }
}

/**
 * @brief Dubins 曲线的路径插值。
 * 
 * 该函数根据给定的运动模式和路径长度，从初始姿态 (x, y, yaw) 计算新的姿态 (new_x, new_y, new_yaw)。
 * 
 * @param mode 运动模式，可以是 DUBINS_L（左转）、DUBINS_S（直行）或 DUBINS_R（右转）。
 * @param length 单步运动路径长度。
 * @param init_pose 初始姿态 (x, y, yaw)。
 * @return new_pose 新的姿态 (new_x, new_y, new_yaw)。
 */
Point3d DubinsCurve::interpolate(int mode, double length, Point3d init_pose)
{
  double x = init_pose.x(), y = init_pose.y(), yaw = init_pose.theta();
  double new_x, new_y, new_yaw;

  if (mode == DUBINS_S){
    new_x = x + length / max_curv_ * cos(yaw);
    new_y = y + length / max_curv_ * sin(yaw);
    new_yaw = yaw;
  } else if (mode == DUBINS_L){
    new_x = x + (sin(yaw + length) - sin(yaw)) / max_curv_;
    new_y = y - (cos(yaw + length) - cos(yaw)) / max_curv_;
    new_yaw = yaw + length;
  } else if (mode == DUBINS_R){
    new_x = x - (sin(yaw - length) - sin(yaw)) / max_curv_;
    new_y = y + (cos(yaw - length) - cos(yaw)) / max_curv_;
    new_yaw = yaw - length;
  } else
    std::cerr << "Error mode" << std::endl;

  return { new_x, new_y, new_yaw };
}

/**
 * @brief 生成 Dubins 曲线的路径。
 * 
 * 该函数根据初始姿态和目标姿态，选择最优的 Dubins 曲线模式，生成平滑的轨迹点。
 * 
 * @param start 初始姿态 (x, y, yaw)。
 * @param goal 目标姿态 (x, y, yaw)。
 * @return path 生成的平滑轨迹点集合。
 */
Points2d DubinsCurve::generation(Point3d start, Point3d goal)
{
    Points2d path;
    double sx = start.x(), sy = start.y(), syaw = start.theta();
    double gx = goal.x(), gy = goal.y(), gyaw = goal.theta();

    // 坐标变换：将目标点平移到原点
    gx -= sx;
    gy -= sy;
    double theta = common::math::mod2pi(atan2(gy, gx));  // 目标点与原点的夹角
    double dist = hypot(gx, gy) * max_curv_;  // 目标点与原点的距离
    double alpha = common::math::mod2pi(syaw - theta);  // 初始姿态与目标方向的夹角
    double beta = common::math::mod2pi(gyaw - theta);  // 目标姿态与目标方向的夹角

    // 选择最优的运动模式
    DubinsMode best_mode;
    double best_cost = std::numeric_limits<double>::max();  // 初始化为最大值
    DubinsLength length;
    DubinsLength best_length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
    DubinsMode mode;

    for (const auto solver : dubins_solvers){
        solver(alpha, beta, dist, length, mode);
        _update(length, mode, best_length, best_mode, best_cost);
    }

    if (best_cost == std::numeric_limits<double>::max())
        return path;  // 如果没有找到有效的路径，返回空路径

    // 插值生成路径点
    int points_num = int(best_cost / step_) + 6;  // 路径点的数量

    std::vector<double> path_x(points_num);
    std::vector<double> path_y(points_num);
    std::vector<double> path_yaw(points_num, alpha);

    std::vector<int> mode_v = { std::get<0>(best_mode), std::get<1>(best_mode), std::get<2>(best_mode) };
    std::vector<double> length_v = { std::get<0>(best_length), std::get<1>(best_length), std::get<2>(best_length) };

    int i = 0;
    for (int j = 0; j < 3; ++j){
        int m = mode_v[j];
        double seg_length = length_v[j];
        // 路径增量
        double d_l = seg_length > 0.0 ? step_ : -step_;
        double x = path_x[i];
        double y = path_y[i];
        double yaw = path_yaw[i];
        // 当前路径长度
        double l = d_l;
        while (fabs(l) <= fabs(seg_length)){
            i += 1;
            auto inter_pose = interpolate(m, l, { x, y, yaw });
            path_x[i] = inter_pose.x(), path_y[i] = inter_pose.y(), path_yaw[i] = inter_pose.theta();
            l += d_l;
        }
        i += 1;
        auto inter_pose = interpolate(m, seg_length, { x, y, yaw });
        path_x[i] = inter_pose.x(), path_y[i] = inter_pose.y(), path_yaw[i] = inter_pose.theta();
    }

    // 移除未使用的数据
    while ((path_x.size() >= 1) && (path_x.back() == 0.0)){
        path_x.pop_back();
        path_y.pop_back();
        path_yaw.pop_back();
    }

    // 坐标变换：将路径点旋转回原始坐标系
    Eigen::AngleAxisd r_vec(theta, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d R = r_vec.toRotationMatrix();
    Eigen::MatrixXd P = Eigen::MatrixXd::Ones(3, path_x.size());

    for (size_t i = 0; i < path_x.size(); ++i){
        P(0, i) = path_x[i];
        P(1, i) = path_y[i];
    }
    P = R * P;

    for (size_t i = 0; i < path_x.size(); ++i){
        path.push_back({ P(0, i) + sx, P(1, i) + sy });
    }

    return path;
}

/**
 * @brief 执行轨迹生成。
 * 
 * 该函数将输入的二维路径点集转换为带有朝向信息的三维姿态点集，
 * 然后调用另一个 `run` 方法生成轨迹。
 * 
 * @param points 输入的路径点集（二维点，包含 x 和 y 坐标）。
 * @param path 输出的生成轨迹。
 * @return 如果轨迹生成成功返回 true，否则返回 false。
 */
bool DubinsCurve::run(const Points2d points, Points2d& path)
{
  if (points.size() < 2){
    return false;
  } else{
    Points3d poses;
    poses.emplace_back(points.begin()->x(), points.begin()->y());
    for (size_t i = 1; i < points.size() - 1; ++i){
      double theta1 = std::atan2(points[i].y() - points[i - 1].y(), points[i].x() - points[i - 1].x());
      double theta2 = std::atan2(points[i + 1].y() - points[i].y(), points[i + 1].x() - points[i].x());
      poses.emplace_back(points[i].x(), points[i].y(), (theta1 + theta2) / 2);
    }
    poses.emplace_back(points.back().x(), points.back().y(), 0);

    return run(poses, path);
  }
}

/**
 * @brief 执行轨迹生成。
 * 
 * 该函数将输入的带有朝向信息的三维路径点集逐段转换为 Dubins 曲线，
 * 并生成平滑的二维轨迹点集。
 * 
 * @param points 输入的路径点集（三维点，包含 x、y 和朝向 theta）。
 * @param path 输出的生成轨迹（二维点集，仅包含 x 和 y 坐标）。
 * @return 如果轨迹生成成功返回 true，否则返回 false。
 */
bool DubinsCurve::run(const Points3d points, Points2d& path)
{
    if (points.size() < 2){
        return false;  // 如果路径点少于 2 个，无法生成轨迹
    } else{
        path.clear();  // 清空输出路径
        for (size_t i = 0; i < points.size() - 1; ++i){
            Points2d path_i = generation(points[i], points[i + 1]);  // 生成每一段的 Dubins 曲线
            if (!path_i.empty())
                path.insert(path.end(), path_i.begin(), path_i.end());  // 将生成的路径点追加到总路径中
        }

        return !path.empty();  // 如果路径不为空，则生成成功
    }
}

void DubinsCurve::setMaxCurv(double max_curv)
{
  assert(max_curv > 0.0);
  max_curv_ = max_curv;
}

void DubinsCurve::setStep(double step)
{
    assert(step > 0.0);
    step_ = step;
}

/**
 * @brief 更新最优运动模式。
 * @param length 当前运动模式的路径长度。
 * @param mode 当前运动模式。
 * @param best_length 目前最优的路径长度。
 * @param best_mode 目前最优的运动模式。
 * @param best_cost 目前最优的路径成本。
 */
void DubinsCurve::_update(DubinsLength length, DubinsMode mode, DubinsLength& best_length, DubinsMode& best_mode, double& best_cost)
{
    if(std::get<0>(length) != DUBINS_NONE){
        double t, p, q;
        std::tie(t, p, q) = length;
        double cost = fabs(t) + fabs(p) + fabs(q);
        if(best_cost > cost){
            best_length = length;
            best_mode = mode;
            best_cost = cost;
        }
    }
}

} // namespace geometry
} // namesapce common


#ifndef MOTION_PLANNING_GEOMETRY_DUBINS_CURVE_HPP_
#define MOTION_PLANNING_GEOMETRY_DUBINS_CURVE_HPP_

#include <functional>
#include "motion_planning/geometry/curve.hpp"

namespace common
{
namespace geometry
{
class DubinsCurve: public Curve
{
private:
    using DubinsMode = std::tuple<int, int, int>;
    using DubinsLength = std::tuple<double, double, double>;

public:
    DubinsCurve();
    DubinsCurve(double step, double max_curv);

    ~DubinsCurve();

    bool run(const Points2d points, Points2d& path);

    bool run(const Points3d points, Points2d& path);

    void LSL(double alpha, double beta, double list, DubinsLength& length, DubinsMode& mode);

    void RSR(double alpha, double beta, double list, DubinsLength& length, DubinsMode& mode);

    void LSR(double alpha, double beta, double list, DubinsLength& length, DubinsMode& mode);

    void RSL(double alpha, double beta, double list, DubinsLength& length, DubinsMode& mode);

    void RLR(double alpha, double beta, double list, DubinsLength& length, DubinsMode& mode);

    void LRL(double alpha, double beta, double list, DubinsLength& length, DubinsMode& mode);

    Point3d interpolate(int mode, double length, Point3d init_pose);

    Points2d generation(Point3d start, Point3d goal);

    void setMaxCurv(double max_curv);

    void setStep(double step);

protected:
    /**
     * @brief 更新最优运动模式。
     * @param length 当前运动模式的路径长度。
     * @param mode 当前运动模式。
     * @param best_length 目前最优的路径长度。
     * @param best_mode 目前最优的运动模式。
     * @param best_cost 目前最优的路径成本。
     */
    void _update(DubinsLength length, DubinsMode mode, DubinsLength& best_length, DubinsMode& best_mode, double& best_cost);

protected:
    double max_curv_;
    double step_;

private:
    using DubinsSolver = std::function<void(double, double, double, DubinsLength&, DubinsMode&)>;
    const std::array<DubinsSolver, 6> dubins_solvers = {
        std::bind(&DubinsCurve::LRL, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                  std::placeholders::_4, std::placeholders::_5),
        std::bind(&DubinsCurve::LSL, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                  std::placeholders::_4, std::placeholders::_5),
        std::bind(&DubinsCurve::LSR, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                  std::placeholders::_4, std::placeholders::_5),
        std::bind(&DubinsCurve::RLR, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                  std::placeholders::_4, std::placeholders::_5),
        std::bind(&DubinsCurve::RSL, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                  std::placeholders::_4, std::placeholders::_5),
        std::bind(&DubinsCurve::RSR, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5)
    };

};
}
}


#endif
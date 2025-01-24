#ifndef COMMON_MOTION_PLANNING_GEOMETRY_BSPLINE_CURVE_HPP_
#define COMMON_MOTION_PLANNING_GEOMETRY_BSPLINE_CURVE_HPP_

#include "motion_planning/geometry/curve.hpp"

namespace common
{
namespace geometry
{
class BsplineCurve: public Curve
{
public:
    BsplineCurve();
    BsplineCurve(double step, int order, int param_mode, int spline_mode);

    ~BsplineCurve();

    double baseFunction(int i, int k, double t, std::vector<double> knot);

    std::vector<double> paramSelection(const Points2d points);

    std::vector<double> knotGeneration(const std::vector<double> param, int n);

    Points2d interpolation(const Points2d points, const std::vector<double> param, const std::vector<double> knot);

    Points2d approximation(const Points2d points, const std::vector<double>param, const std::vector<double> knot);

    Points2d generation(int k, const std::vector<double> knot, Points2d control_pts);

    bool run(const Points2d points, Points2d& path);

    bool run(const Points3d points, Points2d& path);

    void setSplineOrder(int order);

    void setParamMode(int param_mode);

    void setSplineMode(int order);

public:
    enum PARAM_MODE
    {
        UNIFORMSPACED = 0,  // 均匀参数化
        CENTRIPETAL = 1,    // 离心参数化
        CHORDLENGTH = 2,    // 弦长参数化
    };

    enum SPLINE_MODE
    {
        INTERPOLATION = 0,  // 插值方式
        APPROXIMATION = 1,  // 拟合方式
    };

protected:
    int step_;
    int order_;
    int param_mode_;
    int spline_mode_;

};
} // namespace geometry
} // namespace common

#endif
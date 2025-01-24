#include "motion_planning/util/log.hpp"
#include "motion_planning/geometry/curve.hpp"

namespace common
{
namespace geometry
{
/**
 * @brief 构建一个新的Curve对象
 */
Curve::Curve() {};

/**
 * @brief 计算给定路径的长度
 * @param path 需要计算的路径
 * @return 路径的长度
 */
double Curve::length(Points2d path)
{
    double length = 0.0;
    for(size_t i = 0; i < path.size(); ++i){
        //计算欧几里得距离,std::hypot在计算时会自动处理浮点数精度问题，避免溢出
        length += std::hypot(path[i-1].x() - path[i].x(), path[i-1].y() - path[i].y());
    }

    return length;
}

} // namespace geometry
} // namespace common
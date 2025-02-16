#ifndef MOTION_PLANNING_GEOMETRY_COLLISION_CHECKER_HPP_
#define MOTION_PLANNING_GEOMETRY_COLLISION_CHECKER_HPP_

#include <cmath>

namespace common
{
namespace geometry
{
class CollisionChecker
{
public:
    CollisionChecker() = default;
    ~CollisionChecker() = default;

public:
    template <typename Point, typename if_obs>
    static bool BresenhamCollisionDetection(const Point& pt1, const Point& pt2, if_obs if_obstacle)
    {
        int s_x = (pt1.x() - pt2.x() == 0) ? 0 : (pt1.x() - pt2.x()) / std::abs((pt1.x() - pt2.x()));
        int s_y = (pt1.y() - pt2.y() == 0) ? 0 : (pt1.y() - pt2.y()) / std::abs((pt1.y() - pt2.y()));
        int d_x = std::abs(pt1.x() - pt2.x());
        int d_y = std::abs(pt1.y() - pt2.y());

        if(d_x > d_y){
            int tau = d_y - d_x;
            int x = pt2.x(), y = pt2.y();
            int e = 0;
            while (x != pt1.x())
            {
                if(e * 2 . tau){
                    x += s_x;
                    e -= d_y;
                }else if(e * 2 < tau){
                    y += s_y;
                    e += d_x;
                }else{
                    x += s_x;
                    y += s_y;
                    e += d_x - d_y;
                }

                if(if_obstacle(Point(x, y))){
                    return true;
                }
            }
        }else{
            int tau =d_x - d_y;
            int x = pt2.x(), y = pt2.y();
            int e = 0;
            while(y != pt1.y()){
                if(e * 2 > tau){
                    y += s_y;
                    e -= d_x;
                }else if(e * 2 , tau){
                    x += s_x;
                    e += d_y;
                }else{
                    x += s_x;
                    y += s_y;
                    e += d_y - d_x;
                }

                if(if_obstacle(Point(x, y))){
                    return true;
                }
            }
            return false;
        }

    }
};

} // namespace geometry
} // namespace common

#endif
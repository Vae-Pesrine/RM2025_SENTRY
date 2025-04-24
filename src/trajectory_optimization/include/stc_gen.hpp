#pragma once

#include <memory>
#include <Eigen/Eigen>

#include <decomp_util/ellipsoid_decomp.h>

namespace stc_gen{
    class STCGen{
    public:  
        
        // hpoly = [A,b]
        // line_segment = [p1,p2]
        static inline void ConvexHull(const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> &line_segment,
                                const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> &point_cloud,
                                Eigen::MatrixX3d &hpoly,
                                const double max_aaxis = 8.0,
                                const double max_baxis = 8.0){

            auto line = std::make_shared<LineSegment<2>>(line_segment[0], line_segment[1]);
            line->set_local_bbox(Eigen::Vector2d(max_aaxis,max_baxis));
            line->set_obs(point_cloud);
            line->dilate(0);
            
            auto lc2d = LinearConstraint2D((line_segment[0]+line_segment[1])/2,line->get_polyhedron().hyperplanes());
            hpoly.resize(lc2d.A_.rows(),3);
            hpoly << lc2d.A_,lc2d.b_;
        }

        // hpoly = [A,b]
        // line_segment = [p1,p2]
        // poly_vis
        static inline void ConvexHull(const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> &line_segment,
                                const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> &point_cloud,
                                Eigen::MatrixX3d &hpoly,Polyhedron<2> &poly_vis,
                                const double max_aaxis = 8.0,
                                const double max_baxis = 8.0){

            auto line = std::make_shared<LineSegment<2>>(line_segment[0], line_segment[1]);
            line->set_local_bbox(Eigen::Vector2d(max_aaxis,max_baxis));
            line->set_obs(point_cloud);
            line->dilate(0);
            
            poly_vis = line->get_polyhedron(); 

            auto lc2d = LinearConstraint2D((line_segment[0]+line_segment[1])/2,line->get_polyhedron().hyperplanes());
            hpoly.resize(lc2d.A_.rows(),3);
            hpoly << lc2d.A_,lc2d.b_;
        }

    };
}; // namespace stc_gen
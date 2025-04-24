#pragma once

#include <ros/ros.h>
#include <string>

#include <Eigen/Dense>
#include <backward.hpp>
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <grid_map_core/grid_map_core.hpp>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include "minco.hpp"


class CorridorGenerator
{
public:
    template<typename T>
    using VecE = std::vector<T, Eigen::aligned_allocator<T>>;

    int pieceNum_;
    int optDim_;
    int iter;

public:
    CorridorGenerator();
    ~CorridorGenerator();


private:
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    void pathCallback(const nav_msgs::Path::ConstPtr &msg);
    
    void getDiscretePath(double interval = 0.5);

    void optimizePath(const Eigen::Vector2d &goal);

    void getObs();
    void generateCorridor();
    void optimizePathWithCorridor();
    // void displayCorridor();
    void displayDiscretePathPoint();
    void corridor_generate_timer_cb(const ros::TimerEvent &e);
    
    void STCGen(const nav_msgs::OccupancyGridConstPtr map,const std::vector<Eigen::Vector2d> &path,
        VecE<Eigen::MatrixX3d> &hpoly, VecE<Polyhedron<2>> &ploys_vis);
    
    void DecompVel(const double theta, const double vel, double &vx, double &vy);
    
    double Lbfgs(Eigen::VectorXd &xi);
     
    static void positiveSmoothedL1(const double &x, double &f, double &df);
    
    static inline double costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &grad);
    
    void Map2Vec2D(const nav_msgs::OccupancyGridConstPtr costmap,const double center_x, const double center_y, 
        const double roi_w,const double roi_h, VecE<Eigen::Vector2d> &vec_map); 

private:
    double m_length;
    double m_width;
    double m_resolution;

    // ros related
    ros::NodeHandle nh_;

    ros::Subscriber sub_gridmap_;
    ros::Subscriber sub_path_;


    ros::Publisher m_ellipsoid_array_pub;
    ros::Publisher m_polyhedron_array_pub;
    ros::Publisher m_discrete_path_pub;
    ros::Publisher m_optimized_path_pub;
    ros::Timer corridor_generate_timer;
    ros::Publisher pub_opt_path;

    nav_msgs::Path path_;
    nav_msgs::OccupancyGridConstPtr costmap_;

    vec_E<Eigen::Vector2d> m_discrete_path;
    vec_E<Eigen::Vector2d> m_optimized_path;
    // 障碍点
    vec_E<Eigen::Vector2d> m_obs2d;
    EllipsoidDecomp2D m_decomp_util;
    vec_E<Ellipsoid2D> m_elliposid2d;
    vec_E<Polyhedron2D> m_polyhedron2d;
    vec_E<LinearConstraint2D> m_constraints;

    minco::MINCO_S3NU<2> opt_;
    Eigen::VectorXd ts_;

    VecE<Eigen::MatrixX3d> hpolys;

    std::vector<Eigen::Vector2d> path_stc_;

    double max_radius_;
};


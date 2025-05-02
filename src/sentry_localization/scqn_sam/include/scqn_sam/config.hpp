#pragma once

// common
#include <tuple>
#include <vector>
#include <memory>
#include <limits>
#include <iostream>
#include <utility> 

// pcl
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud

// eigen
#include <Eigen/Eigen>

// nano gicp
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <nano_gicp/nano_gicp.hpp>

// quatro
#include <quatro/quatro_module.h>

// scan context
#include <Scancontext.h>

// usr
#include "pose_pcd.hpp"
#include "utilities.hpp"
using PcdPair = std::tuple<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>;

struct NanoGICPConfig
{
    int nano_thread_number_ = 0;
    int nano_correspondences_number_ = 15;
    int nano_max_iter_ = 32;
    int nano_ransac_max_iter_ = 5;
    double max_corr_dist_ = 2.0;
    double icp_score_thre_ = 10.0;
    double transformation_epsilon_ = 0.01;
    double euclidean_fitness_epsilon_ = 0.01;
    double ransac_outlier_rejection_thre_ = 1.0;
};

struct QuatroConfig
{
    bool optimized_matching_en_ = true;
    bool estimat_scale_ = false;
    int quatro_max_num_corres_ = 500;
    int quatro_max_iter_ = 50;
    double quatro_distance_thre_ = 30.0;
    double fpfh_normal_radius_ = 0.30; // It should be 2.5 - 3.0 * `voxel_res`
    double fpfh_radius_ = 0.50;        // It should be 5.0 * `voxel_res`
    double noise_bound_ = 0.30;
    double rot_gnc_factor_ = 1.40;
    double rot_cost_diff_thre_ = 0.0001;
};

struct LoopClosureConfig
{
    bool quatro_en_ = true;
    bool submap_matching_en_ = true;
    int num_submap_keyframes_ = 10;
    double voxel_res_ = 0.1;
    double scancontext_max_correspondence_distance_;
    NanoGICPConfig gicp_config_;
    QuatroConfig quatro_config_;
};

struct  MapMatcherConfig
{
    bool quatro_en_ = false;
    int num_submap_keyframes_ = 10;
    double voxel_res_ = 0.1;
    double scancontext_max_correspondence_distance_;
    NanoGICPConfig gicp_config_;
    QuatroConfig quatro_config_;
};

struct RegistrationOutput
{
    bool is_valid_ = false;
    bool is_converged_ = false;
    double score_ = std::numeric_limits<double>::max();
    Eigen::Matrix4d pose_between_eig_ = Eigen::Matrix4d::Identity();
};




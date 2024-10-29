#ifndef MATCHING_COST_EVALUATER_HPP
#define MATCHING_COST_EVALUATER_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace sentry_localization
{
class MatchingCostEvaluater
{
public:
    MatchingCostEvaluater() {}
    virtual ~MatchingCostEvaluater() {}

    virtual void set_target(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, double max_correspondence_distance) = 0;
    virtual double calc_matching_error(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Matrix4f& transformation, double* inlier_fraction) = 0;
};
}

#endif

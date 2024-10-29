#ifndef GLOBAL_LOCALIZATION_FPFH_TEASER_HPP
#define GLOBAL_LOCALIZATION_FPFH_TEASER_HPP

#include <include/global_localization/engines/global_localization_fpfh_ransac.hpp>
#include <include/global_localization/ransac/matching_cost_evaluater.hpp>

namespace sentry_localization
{
class GlobalLocalizationEngineFPFH_Teaser : public GlobalLocalizationEngineFPFH_RANSAC
{
public:
    GlobalLocalizationEngineFPFH_Teaser(ros::NodeHandle& private_nh);
    virtual ~GlobalLocalizationEngineFPFH_Teaser() override;

    virtual void set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) override;
    virtual GlobalLocalizationResults query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) override;

private:
    using GlobalLocalizationEngineFPFH_RANSAC::private_nh;
    std::unique_ptr<MatchingCostEvaluater> evaluater;
};
}

#endif
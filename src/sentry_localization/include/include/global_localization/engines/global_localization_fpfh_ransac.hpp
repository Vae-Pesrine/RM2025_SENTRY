#ifndef GLOBAL_LOCALIZATION_FPFH_RANSAC_HPP
#define GLOBAL_LOCALIZATION_FPFH_RANSAC_HPP

#include <ros/ros.h>
#include <include/global_localization/engines/global_localization_engine.hpp>
#include <include/global_localization/ransac/ransac_pose_estimation.hpp>


namespace sentry_localization
{
class GlobalLocalizationEngineFPFH_RANSAC : public GlobalLocalizationEngine
{
public :
    GlobalLocalizationEngineFPFH_RANSAC(ros::NodeHandle& private_nh);
    virtual ~GlobalLocalizationEngineFPFH_RANSAC() override;

    virtual void set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) override;
    virtual GlobalLocalizationResults query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) override;

protected:
    //从点云中提取FPFH特征
    pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr extract_fpfh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

    ros::NodeHandle& private_nh;
    std::unique_ptr<RansacPoseEstimation<pcl::FPFHSignature33>> ransac;

    //全局地图的点云和特征
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr global_map;
    pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr global_map_features;
};
}

#endif
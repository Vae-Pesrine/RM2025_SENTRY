#ifndef GLOBAL_LOCALIZATION_BBS_HPP
#define GLOBAL_LOCALIZATION_BBS_HPP

#include <ros/ros.h>
#include <include/global_localization/engines/global_localization_engine.hpp>

namespace sentry_localization
{
class BBSLocalization;
class GlobalLocalizationBBS : public GlobalLocalizationEngine
{
public:
    GlobalLocalizationBBS(ros::NodeHandle& private_nh);
    virtual ~GlobalLocalizationBBS() override;

    virtual void set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) override;

    virtual GlobalLocalizationResults query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) override;

private:
    using Points2D = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;
    //保留在min_z和max_z范围内的点
    Points2D slice(const pcl::PointCloud<pcl::PointXYZ>& cloud, double min_z, double max_z) const;
    //将2D转换为3D点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr unslice(const Points2D& points);

protected:
    ros::NodeHandle& private_nh;
    ros::Publisher pub_gridmap;
    ros::Publisher pub_map_slice;
    ros::Publisher pub_scan_slice;

    std::unique_ptr<BBSLocalization> bbs;
};
}

#endif
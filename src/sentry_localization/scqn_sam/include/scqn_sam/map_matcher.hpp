#pragma once

#include "config.hpp"

class MapMatcher
{
private:
    SCManager sc_manager_;
    nano_gicp::NanoGICP<PointType, PointType> nano_gicp_;
    std::shared_ptr<quatro<PointType>> quatro_handler_ = nullptr;
    int closest_keyframe_index_ = -1;
    pcl::PointCloud<PointType>::Ptr source_cloud_;
    pcl::PointCloud<PointType>::Ptr target_cloud_;
    pcl::PointCloud<PointType> coarse_aligned_;
    pcl::PointCloud<PointType> aligned_;
    MapMatcherConfig config_;

public:
    explicit MapMatcher(const MapMatcherConfig &config);
    ~MapMatcher();

    void updateScanContext(pcl::PointCloud<PointType> cloud);

    int fetchClosestKeyFrameIndex(const PosePcd &front_keyframe,
                                  const std::vector<PosePcdReduced> &saved_map);

    PcdPair setSourceAndTargetCloud(const PosePcd &query_keyframe,
                              const std::vector<PosePcdReduced> &saved_keyframes,
                              const int target_index,
                              const int submap_range,
                              const double voxel_res,
                              const bool quatro_en);

    RegistrationOutput icpAlignment(const pcl::PointCloud<PointType> &source,
                                    const pcl::PointCloud<PointType> &target);

    RegistrationOutput coarseToFineAlignment(const pcl::PointCloud<PointType> &source,
                                             const pcl::PointCloud<PointType> &target);
    
    RegistrationOutput performMapMatcher(const PosePcd &query_keyframe,
                                         const std::vector<PosePcdReduced> &saved_keyframes,
                                         const int closest_keyframe_index);

    pcl::PointCloud<PointType> getSourceCloud();

    pcl::PointCloud<PointType> getTargetCloud();

    pcl::PointCloud<PointType> getCoarseAlignedCloud();

    pcl::PointCloud<PointType> getFinalAlignedCloud();

    int getClosestKeyFrameIndex();
};


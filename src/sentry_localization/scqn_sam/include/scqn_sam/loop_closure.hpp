#pragma once

#include "config.hpp"

class LoopClosure
{
private:
    SCManager sc_manager_;
    nano_gicp::NanoGICP<PointType, PointType> nano_gicp_;
    std::shared_ptr<quatro<PointType>> quatro_handler_ = nullptr;
    int closest_keyframe_idx_ = -1;
    pcl::PointCloud<PointType>::Ptr source_cloud_;
    pcl::PointCloud<PointType>::Ptr target_cloud_;
    pcl::PointCloud<PointType> coarse_aligned_;
    pcl::PointCloud<PointType> aligned_;
    LoopClosureConfig config_;

public:
    explicit LoopClosure(const LoopClosureConfig &config);
    ~LoopClosure();

    void updateScancontext(pcl::PointCloud<PointType> cloud);
    
    int fetchCandidateKeyframeIdx(const PosePcdStamped &query_keyframe,
                                  const std::vector<PosePcdStamped> &keyframes);
    
    PcdPair setSourceAndTargetCloud(const std::vector<PosePcdStamped> &keyframes,
                              const int source_idx,
                              const int target_idx,
                              const int submap_range,
                              const double voxel_res,
                              const bool quatro_en,
                              const bool submap_matching_en);

    RegistrationOutput icpAlignment(const pcl::PointCloud<PointType> &source,
                                    const pcl::PointCloud<PointType> &target);

    RegistrationOutput coarseToFineAlignment(const pcl::PointCloud<PointType> &source,
                                             const pcl::PointCloud<PointType> &target);

    RegistrationOutput performLoopClosure(const PosePcdStamped &query_keyframe,
                                          const std::vector<PosePcdStamped> &keyframes,
                                          const int closest_keyframe_idx);

    pcl::PointCloud<PointType> getSourceCloud();

    pcl::PointCloud<PointType> getTargetCloud();

    pcl::PointCloud<PointType> getCoarseAlignedCloud();

    pcl::PointCloud<PointType> getFinalAlignedCloud();

    int getClosestKeyframeidx();
};
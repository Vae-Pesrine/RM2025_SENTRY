#include "scqn_sam/loop_closure.hpp"

LoopClosure::LoopClosure(
    const LoopClosureConfig &config
) {
    config_ = config;
    const auto &gc = config_.gicp_config_;
    const auto &qc = config_.quatro_config_;
    ////// nano_gicp init
    nano_gicp_.setNumThreads(gc.nano_thread_number_);
    nano_gicp_.setCorrespondenceRandomness(gc.nano_correspondences_number_);
    nano_gicp_.setMaximumIterations(gc.nano_max_iter_);
    nano_gicp_.setRANSACIterations(gc.nano_ransac_max_iter_);
    nano_gicp_.setMaxCorrespondenceDistance(gc.max_corr_dist_);
    nano_gicp_.setTransformationEpsilon(gc.transformation_epsilon_);
    nano_gicp_.setEuclideanFitnessEpsilon(gc.euclidean_fitness_epsilon_);
    nano_gicp_.setRANSACOutlierRejectionThreshold(gc.ransac_outlier_rejection_thre_);
    ////// quatro init
    quatro_handler_ = std::make_shared<quatro<PointType>>(qc.fpfh_normal_radius_,
                                                          qc.fpfh_radius_,
                                                          qc.noise_bound_,
                                                          qc.rot_gnc_factor_,
                                                          qc.rot_cost_diff_thre_,
                                                          qc.quatro_max_iter_,
                                                          qc.estimat_scale_,
                                                          qc.optimized_matching_en_,
                                                          qc.quatro_distance_thre_,
                                                          qc.quatro_max_num_corres_);
    source_cloud_.reset(new pcl::PointCloud<PointType>);
    target_cloud_.reset(new pcl::PointCloud<PointType>);
}


LoopClosure::~LoopClosure() {}


void LoopClosure::updateScancontext(
    pcl::PointCloud<PointType> cloud
) {
    sc_manager_.makeAndSaveScancontextAndKeys(cloud);
}


int LoopClosure::fetchCandidateKeyframeIdx(
    const PosePcdStamped &query_keyframe,
    const std::vector<PosePcdStamped> &keyframes
) {
    // from ScanContext, get the loop candidate
    std::pair<int, float> sc_detected_ = sc_manager_.detectLoopClosureIDGivenScan(query_keyframe.pcd_); // int: nearest node index,
                                                                                                        // float: relative yaw
    int candidate_keyframe_idx = sc_detected_.first;
    if (candidate_keyframe_idx >= 0) // if exists
    {
        // if close enough
        if ((keyframes[candidate_keyframe_idx].pose_corrected_eig_.block<3, 1>(0, 3) - query_keyframe.pose_corrected_eig_.block<3, 1>(0, 3))
                .norm() < config_.scancontext_max_correspondence_distance_)
        {
            return candidate_keyframe_idx;
        }
    }

    return -1;
}

PcdPair LoopClosure::setSourceAndTargetCloud(
    const std::vector<PosePcdStamped> &keyframes,
    const int source_idx,
    const int target_idx,
    const int submap_range,
    const double voxel_res,
    const bool quatro_en,
    const bool submap_matching_en
) {
    pcl::PointCloud<PointType> target_accum, source_accum;
    int num_approx = keyframes[source_idx].pcd_.size() * 2 * submap_range;
    source_accum.reserve(num_approx);
    target_accum.reserve(num_approx);
    if (submap_matching_en){
        for (int i = source_idx - submap_range; i < source_idx + submap_range + 1; ++i){
            if (i >= 0 && i < static_cast<int>(keyframes.size() - 1)){
                source_accum += transformPcd(keyframes[i].pcd_, keyframes[i].pose_corrected_eig_);
            }
        }

        for (int i = target_idx - submap_range; i < target_idx + submap_range + 1; ++i){
            if (i >= 0 && i < static_cast<int>(keyframes.size() - 1)){
                target_accum += transformPcd(keyframes[i].pcd_, keyframes[i].pose_corrected_eig_);
            }
        }
    } else{
        source_accum = transformPcd(keyframes[source_idx].pcd_, keyframes[source_idx].pose_corrected_eig_);
        if (quatro_en){
            target_accum = transformPcd(keyframes[target_idx].pcd_, keyframes[target_idx].pose_corrected_eig_);
        } else{
            // For ICP matching,
            // empirically scan-to-submap matching works better
            for (int i = target_idx - submap_range; i < target_idx + submap_range + 1; ++i){
                if (i >= 0 && i < static_cast<int>(keyframes.size() - 1)){
                    target_accum += transformPcd(keyframes[i].pcd_, keyframes[i].pose_corrected_eig_);
                }
            }
        }
    }

    return {*voxelizePcd(source_accum, voxel_res), *voxelizePcd(target_accum, voxel_res)};
}

RegistrationOutput LoopClosure::icpAlignment(
    const pcl::PointCloud<PointType> &source,
    const pcl::PointCloud<PointType> &target
) {
    RegistrationOutput reg_output;
    aligned_.clear();
    // merge subkeyframes before ICP
    pcl::PointCloud<PointType>::Ptr source_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr target_cloud(new pcl::PointCloud<PointType>());
    *source_cloud = source;
    *target_cloud = target;
    nano_gicp_.setInputSource(source_cloud);
    nano_gicp_.calculateSourceCovariances();
    nano_gicp_.setInputTarget(target_cloud);
    nano_gicp_.calculateTargetCovariances();
    nano_gicp_.align(aligned_);

    // handle results
    reg_output.score_ = nano_gicp_.getFitnessScore();
    // if matchness score is lower than threshold, (lower is better)
    if (nano_gicp_.hasConverged() && reg_output.score_ < config_.gicp_config_.icp_score_thre_){
        reg_output.is_valid_ = true;
        reg_output.is_converged_ = true;
        reg_output.pose_between_eig_ = nano_gicp_.getFinalTransformation().cast<double>();
    }

    return reg_output;
}

RegistrationOutput LoopClosure::coarseToFineAlignment(
    const pcl::PointCloud<PointType> &source,
    const pcl::PointCloud<PointType> &target
) {
    RegistrationOutput reg_output;
    coarse_aligned_.clear();

    reg_output.pose_between_eig_ = (quatro_handler_->align(source, target, reg_output.is_converged_));
    if (!reg_output.is_converged_){
        return reg_output;
    } else{ // if valid
        // coarse align with the result of Quatro
        coarse_aligned_ = transformPcd(source, reg_output.pose_between_eig_);
        const auto &fine_output = icpAlignment(coarse_aligned_, target);
        const auto quatro_tf_ = reg_output.pose_between_eig_;
        reg_output = fine_output;
        reg_output.pose_between_eig_ = fine_output.pose_between_eig_ * quatro_tf_;
    }

    return reg_output;
}

RegistrationOutput LoopClosure::performLoopClosure(
    const PosePcdStamped &query_keyframe,
    const std::vector<PosePcdStamped> &keyframes,
    const int closest_keyframe_idx
) {
    RegistrationOutput reg_output;
    closest_keyframe_idx_ = closest_keyframe_idx;
    if (closest_keyframe_idx_ >= 0){
        // Quatro + NANO-GICP to check loop (from front_keyframe to closest keyframe's neighbor)
        const auto &[source_cloud, target_cloud] = setSourceAndTargetCloud(keyframes,
                                                               query_keyframe.index_,
                                                               closest_keyframe_idx_,
                                                               config_.num_submap_keyframes_,
                                                               config_.voxel_res_,
                                                               config_.quatro_en_,
                                                               config_.submap_matching_en_);
        // Only for visualization
        *source_cloud_ = source_cloud;
        *target_cloud_ = target_cloud;

        if (config_.quatro_en_){
            std::cout << PURPLE_RED << "Execute coarse-to-fine alignment(source, target): " << source_cloud.size()
                      << " vs " << target_cloud.size() << RESET;
            return coarseToFineAlignment(source_cloud, target_cloud);
        } else{
            std::cout << PURPLE_RED << "Execute GICP(source, target): " << source_cloud.size() << " vs "
                      << target_cloud.size() << RESET;
            return icpAlignment(source_cloud, target_cloud);
        }
    } else{
        return reg_output; // dummy output whose `is_valid` is false
    }
}

pcl::PointCloud<PointType> LoopClosure::getSourceCloud()
{
    return *source_cloud_;
}

pcl::PointCloud<PointType> LoopClosure::getTargetCloud()
{
    return *target_cloud_;
}

pcl::PointCloud<PointType> LoopClosure::getCoarseAlignedCloud()
{
    return coarse_aligned_;
}

// NOTE(hlim): To cover ICP-only mode, I just set `Final`, not `Fine`
pcl::PointCloud<PointType> LoopClosure::getFinalAlignedCloud()
{
    return aligned_;
}

int LoopClosure::getClosestKeyframeidx()
{
    return closest_keyframe_idx_;
}
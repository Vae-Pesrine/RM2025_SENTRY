#include <atomic>
#include <vector>
#include <random>
#include <omp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_poly.h>

#include <ros/ros.h>

#include <include/global_localization/ransac/ransac_pose_estimation.hpp>
#include <include/global_localization/ransac/matching_cost_evaluater_flann.hpp>
#include <include/global_localization/ransac/matching_cost_evaluater_voxels.hpp>

namespace sentry_localization {

template <typename FeatureT>
RansacPoseEstimation<FeatureT>::RansacPoseEstimation(ros::NodeHandle& private_nh) : private_nh(private_nh) {}

template <typename FeatureT>
void RansacPoseEstimation<FeatureT>::set_target(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, typename pcl::PointCloud<FeatureT>::ConstPtr target_features) {
  this->target = target;
  this->target_features = target_features;
  feature_tree.reset(new pcl::KdTreeFLANN<FeatureT>);
  feature_tree->setInputCloud(target_features);

  if (private_nh.param<bool>("ransac/voxel_based", true)) {
    evaluater.reset(new MatchingCostEvaluaterVoxels());
  } else {
    evaluater.reset(new MatchingCostEvaluaterFlann());
  }
  evaluater->set_target(target, private_nh.param<double>("ransac/max_correspondence_distance", 1.0));
}

template <typename FeatureT>
void RansacPoseEstimation<FeatureT>::set_source(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, typename pcl::PointCloud<FeatureT>::ConstPtr source_features) {
  this->source = source;
  this->source_features = source_features;
}

template <typename FeatureT>
GlobalLocalizationResults RansacPoseEstimation<FeatureT>::estimate() {
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;

  pcl::registration::CorrespondenceRejectorPoly<pcl::PointXYZ, pcl::PointXYZ> correspondence_rejection;
  correspondence_rejection.setInputTarget(target);
  correspondence_rejection.setInputSource(source);
  correspondence_rejection.setCardinality(3);
  correspondence_rejection.setSimilarityThreshold(private_nh.param<double>("ransac/similarity_threshold", 0.5));

  ROS_INFO_STREAM("RANSAC : Precompute Nearest Features");
  std::vector<std::vector<int>> similar_features(source->size());
#pragma omp parallel for
  for (int i = 0; i < source->size(); i++) {
    std::vector<float> sq_dists;
    feature_tree->nearestKSearch(source_features->at(i), private_nh.param<int>("ransac/correspondence_randomness", 2), similar_features[i], sq_dists);
  }

  std::vector<std::mt19937> mts(omp_get_max_threads());
  for (int i = 0; i < mts.size(); i++) {
    mts[i] = std::mt19937(i * 8191 + i + target->size() + source->size());
  }

  ROS_INFO_STREAM("RANSAC : Main Loop");
  std::atomic_int matching_count(0);
  std::atomic_int iterations(0);
  std::vector<GlobalLocalizationResult::Ptr> results(private_nh.param<int>("ransac/max_iterations", 100000));
  int matching_budget = private_nh.param<int>("ransac/matching_budget", 10000);
  double min_inlier_fraction = private_nh.param<double>("ransac/inlier_fraction", 0.25);

#pragma omp parallel for
  for (int i = 0; i < results.size(); i++) {
    if (matching_count > matching_budget) {
      continue;
    }
    iterations++;

    auto& mt = mts[omp_get_thread_num()];
    std::vector<int> samples;
    std::vector<int> correspondences;
    select_samples(mt, similar_features, samples, correspondences);

    if (!correspondence_rejection.thresholdPolygon(samples, correspondences)) {
      continue;
    }

    Eigen::Matrix4f transformation;
    transformation_estimation.estimateRigidTransformation(*source, samples, *target, correspondences, transformation);

    // if (!params.is_valid(Eigen::Isometry3f(transformation))) {
    //   continue;
    // }

    matching_count++;
    double inlier_fraction = 0.0;
    double matching_error = evaluater->calc_matching_error(*source, transformation, &inlier_fraction);
    ROS_INFO_STREAM("RANSAC : iteration:" << iterations << " matching_count:" << matching_count << " error:" << matching_error << " inlier:" << inlier_fraction);

    if (inlier_fraction > min_inlier_fraction) {
      results[i].reset(new GlobalLocalizationResult(matching_error, inlier_fraction, Eigen::Isometry3f(transformation)));
    }
  }

  return GlobalLocalizationResults(results);
}

template <typename FeatureT>
void RansacPoseEstimation<FeatureT>::select_samples(
  std::mt19937& mt,
  const std::vector<std::vector<int>>& similar_features,
  std::vector<int>& samples,
  std::vector<int>& correspondences) const {
  samples.resize(3);
  for (int i = 0; i < samples.size(); i++) {
    samples[i] = std::uniform_int_distribution<>(0, similar_features.size() - 1)(mt);

    for (int j = 0; j < i; j++) {
      if (samples[j] == samples[i]) {
        i--;
      }
    }
  }

  correspondences.resize(3);
  for (int i = 0; i < samples.size(); i++) {
    if (similar_features[samples[i]].size() == 1) {
      correspondences[i] = similar_features[samples[i]][0];
    } else {
      int idx = std::uniform_int_distribution<>(0, similar_features[samples[i]].size() - 1)(mt);
      correspondences[i] = similar_features[samples[i]][idx];
    }
  }
}

// explicit instantiation
template class RansacPoseEstimation<pcl::FPFHSignature33>;

}
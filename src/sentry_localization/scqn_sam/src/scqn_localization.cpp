#include "scqn_sam/scqn_localization.hpp"

ScqnLocalization::ScqnLocalization(const ros::NodeHandle &n_private):
    nh_(n_private)
{
    ////// ROS params
    // temp vars, only used in constructor
    std::string saved_bag_path;
    double map_match_hz;
    MapMatcherConfig mpmConfig;
    auto &gc = mpmConfig.gicp_config_;
    auto &qc = mpmConfig.quatro_config_;
    // get params
    /* basic */
    nh_.param<std::string>("basic/map_frame", map_frame_, "");
    nh_.param<std::string>("basic/odom_frame", odom_frame_, "");
    nh_.param<std::string>("basic/base_frame", base_frame_, "");
    nh_.param<std::string>("basic/topic_odometry", topic_odometry_, "");
    nh_.param<std::string>("basic/topic_cloud", topic_cloud_, "");

    nh_.param<std::string>("basic/saved_bag_path", saved_bag_path, "");
    nh_.param<double>("basic/map_match_hz", map_match_hz, 1.0);
    nh_.param<double>("basic/visualize_voxel_size", voxel_res_, 1.0);
    /* keyframe */
    nh_.param<double>("keyframe/keyframe_threshold", keyframe_dist_thre_, 1.0);
    nh_.param<int>("keyframe/num_submap_keyframes", mpmConfig.num_submap_keyframes_, 5);
    /* match */
    nh_.param<double>("match/scancontext_max_correspondence_distance",
                      mpmConfig.scancontext_max_correspondence_distance_,
                      15.0);
    nh_.param<double>("match/quatro_nano_gicp_voxel_resolution", mpmConfig.voxel_res_, 0.3);
    /* nano */
    nh_.param<int>("nano_gicp/thread_number", gc.nano_thread_number_, 0);
    nh_.param<double>("nano_gicp/icp_score_threshold", gc.icp_score_thre_, 10.0);
    nh_.param<int>("nano_gicp/correspondences_number", gc.nano_correspondences_number_, 15);
    nh_.param<double>("nano_gicp/max_correspondence_distance", gc.max_corr_dist_, 0.01);
    nh_.param<int>("nano_gicp/max_iter", gc.nano_max_iter_, 32);
    nh_.param<double>("nano_gicp/transformation_epsilon", gc.transformation_epsilon_, 0.01);
    nh_.param<double>("nano_gicp/euclidean_fitness_epsilon", gc.euclidean_fitness_epsilon_, 0.01);
    nh_.param<int>("nano_gicp/ransac/max_iter", gc.nano_ransac_max_iter_, 5);
    nh_.param<double>("nano_gicp/ransac/outlier_rejection_threshold", gc.ransac_outlier_rejection_thre_, 1.0);
    /* quatro */
    nh_.param<bool>("quatro/enable", mpmConfig.quatro_en_, false);
    nh_.param<bool>("quatro/optimize_matching", qc.optimized_matching_en_, true);
    nh_.param<double>("quatro/distance_threshold", qc.quatro_distance_thre_, 30.0);
    nh_.param<int>("quatro/max_correspondences", qc.quatro_max_num_corres_, 200);
    nh_.param<double>("quatro/fpfh_normal_radius", qc.fpfh_normal_radius_, 0.02);
    nh_.param<double>("quatro/fpfh_radius", qc.fpfh_radius_, 0.04);
    nh_.param<bool>("quatro/estimating_scale", qc.estimat_scale_, false);
    nh_.param<double>("quatro/noise_bound", qc.noise_bound_, 0.25);
    nh_.param<double>("quatro/rotation/gnc_factor", qc.rot_gnc_factor_, 0.25);
    nh_.param<double>("quatro/rotation/rot_cost_diff_threshold", qc.rot_cost_diff_thre_, 0.25);
    nh_.param<int>("quatro/rotation/num_max_iter", qc.quatro_max_iter_, 50);

    ////// Matching init
    map_matcher_ = std::make_shared<MapMatcher>(mpmConfig);

    std::string saved_map_path = ros::package::getPath("scqn_sam") + "/" + saved_bag_path;
    ROS_INFO_STREAM(GREEN << saved_map_path << RESET);
    loadMap(saved_map_path);

    corrected_odom_path_.header.frame_id = odom_frame_;
   
    corrected_path_pub_ = nh_.advertise<nav_msgs::Path>("/corrected_path", 10, true);
    corrected_current_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_current_pcd", 10, true);
    
    map_match_pub_ = nh_.advertise<visualization_msgs::Marker>("/map_match", 10, true);
    realtime_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose_stamped", 10);
    saved_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/saved_map", 10, true);
    debug_source_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/src", 10);
    debug_target_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dst", 10);
    debug_coarse_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/coarse_aligned_quatro", 10);
    debug_fine_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fine_aligned_nano_gicp", 10);
    // subscribers
    sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(nh_, topic_odometry_.c_str(), 10);
    sub_pcd_ = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, topic_cloud_.c_str(), 10);
    sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
    sub_odom_pcd_sync_->registerCallback(boost::bind(&ScqnLocalization::odomPcdCallback, this, _1, _2));
    // Timers at the end
    match_timer_ = nh_.createTimer(ros::Duration(1 / map_match_hz), &ScqnLocalization::matchingTimerFunc, this);

    ROS_WARN("Main class, starting node...");
}

// odom and pcd callback
void ScqnLocalization::odomPcdCallback(
    const nav_msgs::OdometryConstPtr &odom_msg, 
    const sensor_msgs::PointCloud2ConstPtr &pcd_msg
) {
    PosePcd current_frame = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_); // to be checked if keyframe or not
    //// 1. realtime pose = last TF * odom
    
    // 回环检测使用
    // 修正之后仍然是odom 到 baselink之间的变换    
    current_frame.pose_corrected_eig_ = last_corrected_TF_ * current_frame.pose_eig_;
    geometry_msgs::PoseStamped current_pose_stamped_ = poseEigToPoseStamped(current_frame.pose_corrected_eig_, odom_frame_);
    realtime_pose_pub_.publish(current_pose_stamped_);
    
    // 修正之后的在odom系下的点云，实际上是对cloud registered进行了修正
    corrected_current_pcd_pub_.publish(pclToPclRos(transformPcd(current_frame.pcd_, current_frame.pose_corrected_eig_), odom_frame_));

    // 保存关键帧 更新odom path
    if (!is_initialized_){ //// init only once
        // 1. save first keyframe
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            last_keyframe_ = current_frame;
        }
        current_keyframe_idx_++;
        //// 2. vis
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            updateOdomsAndPaths(current_frame);
        }
        is_initialized_ = true;
    } else{
        //// 1. check if keyframe
        if (checkIfKeyframe(current_frame, last_keyframe_)){
            // 2. if so, save
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                last_keyframe_ = current_frame;
            }
            current_keyframe_idx_++;
            //// 3. vis
            {
                std::lock_guard<std::mutex> lock(vis_mutex_);
                updateOdomsAndPaths(current_frame);
            }
        }
    }

    return;
}

void ScqnLocalization::matchingTimerFunc(
    const ros::TimerEvent &event
) {
    if (!is_initialized_){
        return;
    }

    //// 1. copy not processed keyframes
    auto t1_ = std::chrono::high_resolution_clock::now();
    PosePcd last_keyframe_copy;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        last_keyframe_copy = last_keyframe_;
        last_keyframe_.processed_ = true;
    }
    if (last_keyframe_copy.index_ == 0 || last_keyframe_copy.processed_){
        return; // already processed or initial keyframe
    }

    //// 2. detect match and calculate TF
    // from last_keyframe_copy keyframe to map (saved keyframes) in threshold radius, get the closest keyframe
    int closest_keyframe_idx = map_matcher_->fetchClosestKeyFrameIndex(last_keyframe_copy, saved_map_from_bag_);
    if (closest_keyframe_idx < 0){
        return; // if no matched candidate
    }
    // Quatro + NANO-GICP to check match (from current_keyframe to closest keyframe in saved map)
    const RegistrationOutput &reg_output = map_matcher_->performMapMatcher(last_keyframe_copy,
                                                                           saved_map_from_bag_,
                                                                           closest_keyframe_idx);

    //// 3. handle corrected results
    if (reg_output.is_valid_){ // TF the pose with the result of match
        ROS_INFO("\033[1;32mMap matching accepted. Score: %.3f\033[0m", reg_output.score_);
        last_corrected_TF_ = reg_output.pose_between_eig_ * last_corrected_TF_; // update TF
        Eigen::Matrix4d TFed_pose = reg_output.pose_between_eig_ * last_keyframe_copy.pose_corrected_eig_;
        // correct poses in vis data
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            corrected_odoms_.points[last_keyframe_copy.index_] = pcl::PointXYZ(TFed_pose(0, 3), TFed_pose(1, 3), TFed_pose(2, 3));
            corrected_odom_path_.poses[last_keyframe_copy.index_] = poseEigToPoseStamped(TFed_pose, odom_frame_);
        }
        // map matches
        matched_pairs_xyz_.push_back({corrected_odoms_.points[last_keyframe_copy.index_], raw_odoms_.points[last_keyframe_copy.index_]}); // for vis
        map_match_pub_.publish(getMatchMarker(matched_pairs_xyz_));
    }
    auto t2_ = std::chrono::high_resolution_clock::now();

    debug_source_pub_.publish(pclToPclRos(map_matcher_->getSourceCloud(), odom_frame_));
    debug_target_pub_.publish(pclToPclRos(map_matcher_->getTargetCloud(), odom_frame_));
    debug_coarse_aligned_pub_.publish(pclToPclRos(map_matcher_->getCoarseAlignedCloud(), odom_frame_));
    debug_fine_aligned_pub_.publish(pclToPclRos(map_matcher_->getFinalAlignedCloud(), odom_frame_));

    // publish odoms and paths
    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        // corrected_odom_pub_.publish(pclToPclRos(corrected_odoms_, odom_frame_));
        corrected_path_pub_.publish(corrected_odom_path_);
    }
    odom_pub_.publish(pclToPclRos(raw_odoms_, odom_frame_));
    // path_pub_.publish(raw_odom_path_);
    // publish saved map
    if (saved_map_vis_switch_ && saved_map_pub_.getNumSubscribers() > 0){
        saved_map_pub_.publish(pclToPclRos(saved_map_pcd_, map_frame_));
        // saved_map_vis_switch_ = false;
    }
    if (!saved_map_vis_switch_ && saved_map_pub_.getNumSubscribers() == 0){
        saved_map_vis_switch_ = true;
    }
    auto t3_ = std::chrono::high_resolution_clock::now();

    double match_time = std::chrono::duration<double, std::milli>(t2_ - t1_).count();
    double vis_time = std::chrono::duration<double, std::milli>(t3_ - t2_).count();
    ROS_INFO_STREAM("Matching: " << match_time << "ms " <<
                    "Vis: "      << vis_time << "ms");

    return;
}

void ScqnLocalization::updateOdomsAndPaths(
    const PosePcd &pose_pcd_in
) {
    corrected_odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, odom_frame_));
    return;
}

visualization_msgs::Marker ScqnLocalization::getMatchMarker(
    const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &match_xyz_pairs
) {
    visualization_msgs::Marker edges_;
    edges_.type = 5u;
    edges_.scale.x = 0.2f;
    edges_.header.frame_id = odom_frame_;
    edges_.pose.orientation.w = 1.0f;
    edges_.color.r = 1.0f;
    edges_.color.g = 1.0f;
    edges_.color.b = 1.0f;
    edges_.color.a = 1.0f;
    for (size_t i = 0; i < match_xyz_pairs.size(); ++i)
    {
        geometry_msgs::Point p_, p2_;
        p_.x = match_xyz_pairs[i].first.x;
        p_.y = match_xyz_pairs[i].first.y;
        p_.z = match_xyz_pairs[i].first.z;
        p2_.x = match_xyz_pairs[i].second.x;
        p2_.y = match_xyz_pairs[i].second.y;
        p2_.z = match_xyz_pairs[i].second.z;
        edges_.points.push_back(p_);
        edges_.points.push_back(p2_);
    }
    return edges_;
}

bool ScqnLocalization::checkIfKeyframe(
    const PosePcd &pose_pcd_in, 
    const PosePcd &latest_pose_pcd
) {
    return keyframe_dist_thre_ < (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3)).norm();
}

void ScqnLocalization::loadMap(
    const std::string &saved_map_path
) {
    rosbag::Bag bag;
    bag.open(saved_map_path, rosbag::bagmode::Read);
    rosbag::View view1(bag, rosbag::TopicQuery("/keyframe_pcd"));
    rosbag::View view2(bag, rosbag::TopicQuery("/keyframe_pose"));
    std::vector<sensor_msgs::PointCloud2> load_pcd_vec;
    std::vector<geometry_msgs::PoseStamped> load_pose_vec;
    for (const rosbag::MessageInstance &pcd_msg : view1){
        sensor_msgs::PointCloud2::ConstPtr pcd_msg_ptr = pcd_msg.instantiate<sensor_msgs::PointCloud2>();
        if (pcd_msg_ptr != nullptr){
            load_pcd_vec.push_back(*pcd_msg_ptr);
        }
    }

    for (const rosbag::MessageInstance &pose_msg : view2){
        geometry_msgs::PoseStamped::ConstPtr pose_msg_ptr = pose_msg.instantiate<geometry_msgs::PoseStamped>();
        if (pose_msg_ptr != nullptr){
            load_pose_vec.push_back(*pose_msg_ptr);
        }
    }

    if (load_pcd_vec.size() != load_pose_vec.size()){
        ROS_ERROR("WRONG BAG FILE!!!!!");
    }

    for (size_t i = 0; i < load_pose_vec.size(); ++i){
        saved_map_from_bag_.push_back(PosePcdReduced(load_pose_vec[i], load_pcd_vec[i], i));
        saved_map_pcd_ += transformPcd(saved_map_from_bag_[i].pcd_, saved_map_from_bag_[i].pose_eig_);
        map_matcher_->updateScanContext(saved_map_from_bag_[i].pcd_); // note: update scan context for loop candidate detection
    }

    saved_map_pcd_ = *voxelizePcd(saved_map_pcd_, voxel_res_);
    bag.close();
    return;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "scqn_localization_node");
    ros::NodeHandle pr_nh("~");

    ScqnLocalization scqn_localization(pr_nh);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
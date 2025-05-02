#include "scqn_sam/scqn_sam.hpp"
ScqnSam::ScqnSam(const ros::NodeHandle &pr_nh):
    nh_(pr_nh) {
    ////// ROS params
    double loop_update_hz, vis_hz;
    LoopClosureConfig lc_config;
    auto &gc = lc_config.gicp_config_;
    auto &qc = lc_config.quatro_config_;
    /* basic */
    nh_.param<std::string>("basic/map_frame", map_frame_, "map");
    nh_.param<std::string>("basic/base_frame", base_frame_, "base_link");
    nh_.param<double>("basic/loop_update_hz", loop_update_hz, 1.0);
    nh_.param<double>("basic/vis_hz", vis_hz, 0.5);
    nh_.param<double>("save_voxel_resolution", voxel_res_, 0.3);
    nh_.param<double>("quatro_nano_gicp_voxel_resolution", lc_config.voxel_res_, 0.3);
    /* sub topic */
    nh_.param<std::string>("sub_topic/odometry", topic_odometry_, "");
    nh_.param<std::string>("sub_topic/cloud", topic_cloud_, "");
    /* keyframe */
    nh_.param<double>("keyframe/keyframe_threshold", keyframe_thre_, 1.0);
    nh_.param<int>("keyframe/num_submap_keyframes", lc_config.num_submap_keyframes_, 5);
    nh_.param<bool>("keyframe/submap_matching_en", lc_config.submap_matching_en_, false);
    /* ScanContext */
    nh_.param<double>("scancontext_max_correspondence_distance",
                      lc_config.scancontext_max_correspondence_distance_,
                      35.0);
    /* nano (GICP config) */
    nh_.param<int>("nano_gicp/thread_number", gc.nano_thread_number_, 0);
    nh_.param<double>("nano_gicp/icp_score_threshold", gc.icp_score_thre_, 10.0);
    nh_.param<int>("nano_gicp/correspondences_number", gc.nano_correspondences_number_, 15);
    nh_.param<double>("nano_gicp/max_correspondence_distance", gc.max_corr_dist_, 0.01);
    nh_.param<int>("nano_gicp/max_iter", gc.nano_max_iter_, 32);
    nh_.param<double>("nano_gicp/transformation_epsilon", gc.transformation_epsilon_, 0.01);
    nh_.param<double>("nano_gicp/euclidean_fitness_epsilon", gc.euclidean_fitness_epsilon_, 0.01);
    nh_.param<int>("nano_gicp/ransac/max_iter", gc.nano_ransac_max_iter_, 5);
    nh_.param<double>("nano_gicp/ransac/outlier_rejection_threshold", gc.ransac_outlier_rejection_thre_, 1.0);
    /* quatro (Quatro config) */
    nh_.param<bool>("quatro/en", lc_config.quatro_en_, false);
    nh_.param<bool>("quatro/optimize_matching_en", qc.optimized_matching_en_, true);
    nh_.param<double>("quatro/distance_threshold", qc.quatro_distance_thre_, 30.0);
    nh_.param<int>("quatro/max_num_correspondences", qc.quatro_max_num_corres_, 200);
    nh_.param<double>("quatro/fpfh_normal_radius", qc.fpfh_normal_radius_, 0.3);
    nh_.param<double>("quatro/fpfh_radius", qc.fpfh_radius_, 0.5);
    nh_.param<bool>("quatro/estimating_scale", qc.estimat_scale_, false);
    nh_.param<double>("quatro/noise_bound", qc.noise_bound_, 0.3);
    nh_.param<double>("quatro/rotation/gnc_factor", qc.rot_gnc_factor_, 1.4);
    nh_.param<double>("quatro/rotation/rot_cost_diff_threshold", qc.rot_cost_diff_thre_, 0.0001);
    nh_.param<int>("quatro/rotation/num_max_iter", qc.quatro_max_iter_, 50);
    /* results */
    nh_.param<bool>("result/bag/save_en", save_map_bag_, false);
    nh_.param<bool>("result/pcd/save_en", save_map_pcd_, false);
    nh_.param<std::string>("result/bag/dir", bag_dir_, "");
    nh_.param<std::string>("result/bag/name", bag_name_, "");
    nh_.param<std::string>("result/pcd/dir", pcd_dir_, "");
    nh_.param<std::string>("result/pcd/name", pcd_name_, "");

    ROS_INFO_STREAM(GREEN << "map frame: " << map_frame_ << RESET);

    loop_closure_.reset(new LoopClosure(lc_config));
    /* Initialization of GTSAM */
    gtsam::ISAM2Params isam_params_;
    isam_params_.relinearizeThreshold = 0.01;
    isam_params_.relinearizeSkip = 1;
    isam_handler_ = std::make_shared<gtsam::ISAM2>(isam_params_);
    /* ROS things */
    odom_path_.header.frame_id = map_frame_;
    corrected_path_.header.frame_id = map_frame_;
    package_path_ = ros::package::getPath("scqn_sam");
    /* publishers */
    odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ori_odom", 10, true);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/ori_path", 10, true);
    corrected_odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_odom", 10, true);
    corrected_path_pub_ = nh_.advertise<nav_msgs::Path>("/corrected_path", 10, true);
    corrected_pcd_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_map", 10, true);
    corrected_current_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_current_pcd", 10, true);
    loop_detection_pub_ = nh_.advertise<visualization_msgs::Marker>("/loop_detection", 10, true);
    realtime_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose_stamped", 10);
    debug_source_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/src", 10, true);
    debug_target_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dst", 10, true);
    debug_coarse_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/coarse_aligned_quatro", 10, true);
    debug_fine_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fine_aligned_nano_gicp", 10, true);
    /* subscribers */
    sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(nh_, topic_odometry_.c_str(), 10);
    sub_pcd_ = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, topic_cloud_.c_str(), 10);
    sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
    sub_odom_pcd_sync_->registerCallback(boost::bind(&ScqnSam::odomPcdCallback, this, _1, _2));
    sub_save_flag_ = nh_.subscribe("/save_dir", 1, &ScqnSam::saveFlagCallback, this);
    /* Timers */
    loop_timer_ = nh_.createTimer(ros::Duration(1 / loop_update_hz), &ScqnSam::loopTimerCallback, this);
    vis_timer_ = nh_.createTimer(ros::Duration(1 / vis_hz), &ScqnSam::visTimerCallback, this);
    ROS_INFO("Main class, starting node...");
}

void ScqnSam::odomPcdCallback(
    const nav_msgs::OdometryConstPtr &odom_msg,
    const sensor_msgs::PointCloud2ConstPtr &pcd_msg
) {
    Eigen::Matrix4d last_odom_tf;
    last_odom_tf = current_frame_.pose_eig_;                              // to calculate delta
    current_frame_ = PosePcdStamped(*odom_msg, *pcd_msg, current_keyframe_idx_); // to be checked if keyframe or not
    auto t1 = std::chrono::high_resolution_clock::now();
    {
        //// 1. realtime pose = last corrected odom * delta (last -> current)
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        odom_delta_ = odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_;
        current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;
        realtime_pose_pub_.publish(poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_));
        tf_broadcaster_.sendTransform(tf::StampedTransform(poseEigToROSTf(current_frame_.pose_corrected_eig_),
                                                        ros::Time::now(),
                                                        map_frame_,
                                                        base_frame_));
    }
    corrected_current_pcd_pub_.publish(pclToPclRos(transformPcd(current_frame_.pcd_, current_frame_.pose_corrected_eig_), map_frame_));

    if (!is_initialized_) //// init only once
    {
        // others
        keyframes_.push_back(current_frame_);
        updateOdomsAndPaths(current_frame_);
        // graph
        auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished(); // rad*rad,
                                                                                                    // meter*meter
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, poseEigToGtsamPose(current_frame_.pose_eig_), prior_noise));
        init_esti_.insert(current_keyframe_idx_, poseEigToGtsamPose(current_frame_.pose_eig_));
        current_keyframe_idx_++;
        // ScanContext
        loop_closure_->updateScancontext(current_frame_.pcd_);
        is_initialized_ = true;
    }
    else
    {
        //// 2. check if keyframe
        auto t2 = std::chrono::high_resolution_clock::now();
        if (checkIfKeyframe(current_frame_, keyframes_.back()))
        {
            // 2-2. if so, save
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                keyframes_.push_back(current_frame_);
            }
            // 2-3. if so, add to graph
            auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
            gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
            gtsam::Pose3 pose_from = poseEigToGtsamPose(keyframes_[current_keyframe_idx_ - 1].pose_corrected_eig_);
            gtsam::Pose3 pose_to = poseEigToGtsamPose(current_frame_.pose_corrected_eig_);
            {
                std::lock_guard<std::mutex> lock(graph_mutex_);
                gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(current_keyframe_idx_ - 1,
                                                                    current_keyframe_idx_,
                                                                    pose_from.between(pose_to),
                                                                    odom_noise));
                init_esti_.insert(current_keyframe_idx_, pose_to);
            }
            current_keyframe_idx_++;
            // 2-4. if so, update ScanContext
            loop_closure_->updateScancontext(current_frame_.pcd_);

            //// 3. vis
            auto t3 = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(vis_mutex_);
                updateOdomsAndPaths(current_frame_);
            }

            //// 4. optimize with graph
            auto t4 = std::chrono::high_resolution_clock::now();
            // m_corrected_esti = gtsam::LevenbergMarquardtOptimizer(m_gtsam_graph, init_esti_).optimize(); // cf. isam.update vs values.LM.optimize
            {
                std::lock_guard<std::mutex> lock(graph_mutex_);
                isam_handler_->update(gtsam_graph_, init_esti_);
                isam_handler_->update();
                if (loop_added_flag_){ // https://github.com/TixiaoShan/LIO-SAM/issues/5#issuecomment-653752936
                    isam_handler_->update();
                    isam_handler_->update();
                    isam_handler_->update();
                }
                gtsam_graph_.resize(0);
                init_esti_.clear();
            }

            //// 5. handle corrected results
            // get corrected poses and reset odom delta (for realtime pose pub)
            auto t5 = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
                corrected_esti_ = isam_handler_->calculateEstimate();
                last_corrected_pose_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size() - 1));
                odom_delta_ = Eigen::Matrix4d::Identity();
            }
            // correct poses in keyframes
            if (loop_added_flag_){
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                for (size_t i = 0; i < corrected_esti_.size(); ++i){
                    keyframes_[i].pose_corrected_eig_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
                }
                loop_added_flag_ = false;
            }
            auto t6 = std::chrono::high_resolution_clock::now();
            
            auto real_time = std::chrono::duration<double, std::milli>(t2 - t1).count();
            auto key_add_time = std::chrono::duration<double, std::milli>(t3 - t2).count();
            auto vis_time = std::chrono::duration<double, std::milli>(t4 - t3).count();
            auto opt_time = std::chrono::duration<double, std::milli>(t5 - t4).count();
            auto res_time = std::chrono::duration<double, std::milli>(t6 - t5).count();
            auto tot_time = std::chrono::duration<double, std::milli>(t6 - t1).count();
            ROS_INFO_STREAM("real: "    << real_time    << "ms " <<
                            "key_add: " << key_add_time << "ms " <<
                            "vis: "     << vis_time     << "ms " <<
                            "opt: "     << opt_time     << "ms " <<
                            "res: "     << res_time     << "ms " <<
                            "tot: "     << tot_time     << "ms ");
        }
    }

    return;
}

void ScqnSam::loopTimerCallback(
    const ros::TimerEvent &event
) {
    auto &latest_keyframe = keyframes_.back();
    if (!is_initialized_ || keyframes_.empty() || latest_keyframe.processed_){
        return;
    }
    latest_keyframe.processed_ = true;

    auto t1 = std::chrono::high_resolution_clock::now();
    const int closest_keyframe_idx = loop_closure_->fetchCandidateKeyframeIdx(latest_keyframe, keyframes_);
    if (closest_keyframe_idx < 0){
        return;
    }

    const RegistrationOutput &reg_output = loop_closure_->performLoopClosure(latest_keyframe, keyframes_, closest_keyframe_idx);
    if (reg_output.is_valid_){
        ROS_INFO("\033[1;32mLoop closure accepted. Score: %.3f\033[0m", reg_output.score_);
        const auto &score = reg_output.score_;
        gtsam::Pose3 pose_from = poseEigToGtsamPose(reg_output.pose_between_eig_ * latest_keyframe.pose_corrected_eig_); // IMPORTANT: take care of the order
        gtsam::Pose3 pose_to = poseEigToGtsamPose(keyframes_[closest_keyframe_idx].pose_corrected_eig_);
        auto variance_vector = (gtsam::Vector(6) << score, score, score, score, score, score).finished();
        gtsam::noiseModel::Diagonal::shared_ptr loop_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(latest_keyframe.index_,
                                                                closest_keyframe_idx,
                                                                pose_from.between(pose_to),
                                                                loop_noise));
        }
        loop_idx_pairs_.push_back({latest_keyframe.index_, closest_keyframe_idx}); // for vis
        loop_added_flag_vis_ = true;
        loop_added_flag_ = true;
    } else{
        ROS_WARN("Loop closure rejected. Score: %.3f", reg_output.score_);
    }
    auto t2 = std::chrono::high_resolution_clock::now();

    debug_source_pub_.publish(pclToPclRos(loop_closure_->getSourceCloud(), map_frame_));
    debug_target_pub_.publish(pclToPclRos(loop_closure_->getTargetCloud(), map_frame_));
    debug_fine_aligned_pub_.publish(pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_));
    debug_coarse_aligned_pub_.publish(pclToPclRos(loop_closure_->getCoarseAlignedCloud(), map_frame_));

    ROS_INFO("loop: %.1f", std::chrono::duration<double, std::milli>(t2 - t1).count() / 1e3);
    return;
}

void ScqnSam::visTimerCallback(
    const ros::TimerEvent &event
) {
    if (!is_initialized_)
    {
        return;
    }

    auto tv1 = std::chrono::high_resolution_clock::now();
    //// 1. if loop closed, correct vis data
    if (loop_added_flag_vis_){ // copy and ready
        gtsam::Values corrected_esti_copied;
        pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
        nav_msgs::Path corrected_path;
        {
            std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
            corrected_esti_copied = corrected_esti_;
        }
        // correct pose and path
        for (size_t i = 0; i < corrected_esti_copied.size(); ++i){
            gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
            corrected_odoms.points.emplace_back(pose_.translation().x(), pose_.translation().y(), pose_.translation().z());
            corrected_path.poses.push_back(gtsamPoseToPoseStamped(pose_, map_frame_));
        }
        // update vis of loop constraints
        if (!loop_idx_pairs_.empty()){
            loop_detection_pub_.publish(getLoopMarkers(corrected_esti_copied));
        }
        // update with corrected data
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            corrected_odoms_ = corrected_odoms;
            corrected_path_.poses = corrected_path.poses;
        }
        loop_added_flag_vis_ = false;
    }
    //// 2. publish odoms, paths
    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        odom_pub_.publish(pclToPclRos(odoms_, map_frame_));
        path_pub_.publish(odom_path_);
        corrected_odom_pub_.publish(pclToPclRos(corrected_odoms_, map_frame_));
        corrected_path_pub_.publish(corrected_path_);
    }

    //// 3. global map
    if (global_map_vis_switch_ && corrected_pcd_map_pub_.getNumSubscribers() > 0){ // save time, only once
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        corrected_pcd_map_pub_.publish(pclToPclRos<PointType>(*voxelized_map, map_frame_));
        global_map_vis_switch_ = false;
    }

    if(!global_map_vis_switch_ && corrected_pcd_map_pub_.getNumSubscribers() == 0){
        global_map_vis_switch_ = true;
    }
    auto tv2 = std::chrono::high_resolution_clock::now();
    ROS_INFO("vis: %.1fms", std::chrono::duration<double, std::milli>(tv2 - tv1).count() / 1e3);
    
    return;
}

void ScqnSam::saveFlagCallback(
    const std_msgs::String::ConstPtr &msg
) {
    if (save_map_bag_) {
        std::string dir_path = package_path_ + bag_dir_;
        std::string file_path = dir_path + "/" + bag_name_;
    
        // 保证目录存在
        if (!fs::exists(dir_path)) {
            fs::create_directories(dir_path);
        }
    
        // 保证文件存在（如不存在则创建空文件）
        if (!fs::exists(file_path)) {
            std::ofstream ofs(file_path);
            ofs.close();
        }
    
        rosbag::Bag bag;
        bag.open(file_path, rosbag::bagmode::Write);
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                ros::Time time;
                time.fromSec(keyframes_[i].timestamp_);
                bag.write("/keyframe_pcd", time, pclToPclRos(keyframes_[i].pcd_, map_frame_));
                bag.write("/keyframe_pose", time, poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_));
            }
        }
        bag.close();
        ROS_INFO_STREAM(BLUE << "Result saved in .bag format in file: " << file_path << RESET);
    }

    if (save_map_pcd_) {
        std::string dir_path = package_path_ + pcd_dir_;
        std::string file_path = dir_path + "/" + pcd_name_;
    
        // 保证目录存在
        if (!fs::exists(dir_path)) {
            fs::create_directories(dir_path);
        }
    
        // 保证文件存在（如不存在则创建空文件）
        if (!fs::exists(file_path)) {
            std::ofstream ofs(file_path);
            ofs.close();
        }
    
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        pcl::io::savePCDFileASCII<PointType>(file_path, *voxelized_map);
        ROS_INFO_STREAM(BLUE << "Accumulated map cloud saved in .pcd format in file: " << file_path << RESET);
    }
}

ScqnSam::~ScqnSam()
{
    save();
}

void ScqnSam::updateOdomsAndPaths(const PosePcdStamped &pose_pcd_in)
{
    odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                               pose_pcd_in.pose_eig_(1, 3),
                               pose_pcd_in.pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                         pose_pcd_in.pose_corrected_eig_(1, 3),
                                         pose_pcd_in.pose_corrected_eig_(2, 3));
    odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
    corrected_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
    return;
}

visualization_msgs::Marker ScqnSam::getLoopMarkers(const gtsam::Values &corrected_esti_in)
{
    visualization_msgs::Marker edges;
    edges.type = 5u;
    edges.scale.x = 0.12f;
    edges.header.frame_id = map_frame_;
    edges.pose.orientation.w = 1.0f;
    edges.color.r = 1.0f;
    edges.color.g = 1.0f;
    edges.color.b = 1.0f;
    edges.color.a = 1.0f;
    for (size_t i = 0; i < loop_idx_pairs_.size(); ++i)
    {
        if (loop_idx_pairs_[i].first >= corrected_esti_in.size() ||
            loop_idx_pairs_[i].second >= corrected_esti_in.size())
        {
            continue;
        }
        gtsam::Pose3 pose = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].first);
        gtsam::Pose3 pose2 = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].second);
        geometry_msgs::Point p, p2;
        p.x = pose.translation().x();
        p.y = pose.translation().y();
        p.z = pose.translation().z();
        p2.x = pose2.translation().x();
        p2.y = pose2.translation().y();
        p2.z = pose2.translation().z();
        edges.points.push_back(p);
        edges.points.push_back(p2);
    }
    return edges;
}

bool ScqnSam::checkIfKeyframe(const PosePcdStamped &pose_pcd_in, const PosePcdStamped &latest_pose_pcd)
{
    return keyframe_thre_ < (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3)).norm();
}

void ScqnSam::save(){
    if (save_map_bag_) {
        std::string dir_path = package_path_ + "/" + bag_dir_;
        std::string file_path = dir_path + "/" + bag_name_;
    
        // 保证目录存在
        if (!fs::exists(dir_path)) {
            fs::create_directories(dir_path);
        }
    
        // 保证文件存在（如不存在则创建空文件）
        if (!fs::exists(file_path)) {
            std::ofstream ofs(file_path);
            ofs.close();
        }
    
        rosbag::Bag bag;
        bag.open(file_path, rosbag::bagmode::Write);
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                ros::Time time;
                time.fromSec(keyframes_[i].timestamp_);
                bag.write("/keyframe_pcd", time, pclToPclRos(keyframes_[i].pcd_, map_frame_));
                bag.write("/keyframe_pose", time, poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_));
            }
        }
        bag.close();
        ROS_INFO_STREAM(BLUE << "Result saved in .bag format in file: " << file_path << RESET);
    }

    if (save_map_pcd_) {
        std::string dir_path = package_path_ + "/" + pcd_dir_;
        std::string file_path = dir_path + "/" + pcd_name_;
    
        // 保证目录存在
        if (!fs::exists(dir_path)) {
            fs::create_directories(dir_path);
        }
    
        // 保证文件存在（如不存在则创建空文件）
        if (!fs::exists(file_path)) {
            std::ofstream ofs(file_path);
            ofs.close();
        }
    
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        pcl::io::savePCDFileASCII<PointType>(file_path, *voxelized_map);
        ROS_INFO_STREAM(BLUE << "Accumulated map cloud saved in .pcd format in file: " << file_path << RESET);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scqn_sam_node");
    ros::NodeHandle pr_nh("~");

    ScqnSam scqn_sam(pr_nh);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();


    return 0;
}

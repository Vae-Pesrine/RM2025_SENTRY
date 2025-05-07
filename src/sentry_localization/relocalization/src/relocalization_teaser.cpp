#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pointcloud_iof/pcd_loader.hpp>
#include <pointcloud_iof/pcl_eigen_converter.hpp>
#include <pointcloud_iof/gravity_alignment.hpp>
#include <pcl/filters/voxel_grid.h>
#include <relocalization/relocalization.hpp>



Relocalization::Relocalization(): 
    pr_nh_("~"),
    first_reconfigure_call_(true),
    if_global_localization_(true),
    initial_transform_(Eigen::Isometry3d::Identity()),
    result_t_(Eigen::Isometry3d::Identity()), 
    prior_result_t_(Eigen::Isometry3d::Identity()){    
    /* basic */
    pr_nh_.param<bool>("basic/debug_en", debug_en_, false);
    pr_nh_.param<bool>("basic/pub_prior_pcd_en", pub_prior_pcd_en_, false);
    pr_nh_.param<std::string>("basic/map_frame", map_frame_, "");
    pr_nh_.param<std::string>("basic/odom_frame", odom_frame_, "");
    pr_nh_.param<std::string>("basic/base_frame", base_frame_, "");
    pr_nh_.param<std::string>("basic/lidar_frame", lidar_frame_, "");
    pr_nh_.param<std::string>("basic/pcd_in_topic", pcd_in_topic_, "");
    pr_nh_.param<std::string>("basic/imu_in_topic", imu_in_topic_, "");
    pr_nh_.param<std::string>("basic/initialpose_in_topic", initialpose_in_topic_, "");

    /* small gicp */
    pr_nh_.param<int>("gicp/num_threads", num_threads_, 4);
    pr_nh_.param<int>("gicp/num_neighbors", num_neighbors_, 20);
    pr_nh_.param<float>("gicp/max_dist_sq", max_dist_sq_, 1.0);
    pr_nh_.param<int>("gicp/max_iterations", max_iterations_, 30);
    pr_nh_.param<int>("gicp/align_time_threshold", align_time_thre_, 10);
    pr_nh_.param<float>("gicp/global_leaf_size", global_leaf_size_, 0.25);
    pr_nh_.param<float>("gicp/registered_leaf_size", registered_leaf_size_, 1.0);

    /* prior pcd */
    pr_nh_.param<double>("pcd/x", x_, 0.0);
    pr_nh_.param<double>("pcd/y", y_, 0.0);
    pr_nh_.param<double>("pcd/z", z_, 0.0);
    pr_nh_.param<double>("pcd/yaw", yaw_, 0.0);
    pr_nh_.param<double>("pcd/roll", roll_, 0.0);
    pr_nh_.param<double>("pcd/pitch", pitch_, 0.0);
    pr_nh_.param<std::string>("pcd/prior_pcd_file", prior_pcd_file_, "");

    /* timer */
    pr_nh_.param<double>("timer/registration_frequency", registration_frequency_, 4);
    pr_nh_.param<double>("timer/pose_update_frequency", pose_update_frequency_, 2);
    pr_nh_.param<double>("timer/transform_tolerance", transform_tolerance_, 0.18);

    /* gpu bbs3d */
    pr_nh_.param<int>("gpu_bbs/max_level", max_level_, 6);
    pr_nh_.param<double>("gpu_bbs/min_level_resolution", min_level_res_, 0.5);
    pr_nh_.param<float>("gpu_bbs/map_leaf_size", map_leaf_size_, 0.2);
    pr_nh_.param<float>("gpu_bbs/source_leaf_size", source_leaf_size_, 0.1);
    pr_nh_.param<double>("gpu_bbs/min_scan_range", min_scan_range_, 0.0);
    pr_nh_.param<double>("gpu_bbs/max_scan_range", max_scan_range_, 100.0);
    pr_nh_.param<double>("gpu_bbs/score_percentage_threshold", score_percentage_thre_, 0.9);
    pr_nh_.param<int>("gpu_bbs/timeout_msec", timeout_msec_, 0);
    std::vector<double> min_rpy_vec, max_rpy_vec;    
    auto toEigen = [](const std::vector<double>& vec) -> Eigen::Vector3d {
        Eigen::Vector3d e_vec;
        for(int i = 0; i < 3; ++i){
            e_vec[i] = (vec[i] == 6.28) ? 2 * M_PI : vec[i];
        }
        return e_vec;
    };
    pr_nh_.param<std::vector<double>>("gpu_bbs/min_rpy", min_rpy_vec, std::vector<double>{0.0, 0.0, 0.0}); 
    pr_nh_.param<std::vector<double>>("gpu_bbs/max_rpy", max_rpy_vec, std::vector<double>{0.0, 0.0, 0.0}); 
    min_rpy_ = toEigen(min_rpy_vec);
    max_rpy_ = toEigen(max_rpy_vec);

    /* teaser++ */
    pr_nh_.param<double>("teaser++/noise_bound", noise_bound_, 0.01);
    pr_nh_.param<double>("teaser++/cbar2", cbar2_, 1);
    pr_nh_.param<bool>("teaser++/estimate_scaling", estimate_scaling_, false);
    pr_nh_.param<int>("teaser++/iterations", teaser_iterations_, 100);


    /* ros */
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    sub_pcd_ = nh_.subscribe<sensor_msgs::PointCloud2>
        (pcd_in_topic_.c_str(), 10, &Relocalization::registerPcdCallback, this);
    sub_imu_ = nh_.subscribe<sensor_msgs::Imu>
        (imu_in_topic_.c_str(), 10, &Relocalization::imuCallback, this);
    sub_initialpose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>
       (initialpose_in_topic_.c_str(), 10, &Relocalization::initialPoseCallback, this);
    pub_pcd_ = nh_.advertise<sensor_msgs::PointCloud2>("/prior_pcd", 10, this);

    register_timer_ = this->createWallTimer
        (ros::WallDuration(1 / registration_frequency_), &Relocalization::runRegistration, this, false, true);
    transform_timer_ = this->createWallTimer
        (ros::WallDuration(1 / pose_update_frequency_), &Relocalization::publishTransform, this, false, true);

    /* reconfigure */
    cfg_server_ = std::make_unique<dynamic_reconfigure::Server<relocalization::RelocalizationCfgConfig>>();
    cfg_server_->setCallback(boost::bind(&Relocalization::dynamicReconfigCallback, this, _1, _2));

    global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    registered_scan_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    setInitialTransformation();
    loadGlobalMap(prior_pcd_file_);

    /* small gicp */
    gicp_register_ = std::make_unique<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
    target_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>
        (*global_map_, global_leaf_size_);
    small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(target_, small_gicp::KdTreeBuilderOMP(num_threads_));
    gicp_register_->reduction.num_threads = num_threads_;
    gicp_register_->rejector.max_dist_sq = max_dist_sq_;
    gicp_register_->optimizer.max_iterations = max_iterations_;
    std::cout << BLUE << "Gicp init..." << RESET;
    
    /* gpu bbs3d */
    gpu_bbs3d_ = std::make_unique<gpu::BBS3D>();
    filtered_map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    filtered_scan_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if(map_leaf_size_ != 0.0f){
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
        filter.setInputCloud(global_map_);
        filter.filter(*filtered_map_cloud_);
    } else{
        *filtered_map_cloud_ = *global_map_;
    }
    pciof::pcl_to_eigen(filtered_map_cloud_, map_points);
    std::cout << GREEN << "BBS global map: " << map_points.size() << RESET;
    /* don't change the initialization sequence */
    gpu_bbs3d_->set_tar_points(map_points, min_level_res_, max_level_);
    gpu_bbs3d_->set_trans_search_range(map_points);
    gpu_bbs3d_->set_angular_search_range(min_rpy_.cast<float>(), max_rpy_.cast<float>());
    gpu_bbs3d_->set_score_threshold_percentage(static_cast<float>(score_percentage_thre_));
    if(timeout_msec_ > 0){
        gpu_bbs3d_->enable_timeout();
        gpu_bbs3d_->set_timeout_duration_in_msec(timeout_msec_);
    } 
    std::cout << BLUE << "Gpu bbs init ..." << RESET;

    /* teaser++ */
    quatro_params_ = std::make_unique<teaser::RobustRegistrationSolver::Params>();
    quatro_fpfh_ = std::make_shared<teaser::FPFHEstimation>();
    quatro_source_ = std::make_shared<teaser::PointCloud>();
    quatro_target_ = std::make_shared<teaser::PointCloud>();
    quatro_source_feature_.reset(new teaser::FPFHCloud);
    quatro_target_feature_.reset(new teaser::FPFHCloud);


    quatro_params_->noise_bound = noise_bound_;
    quatro_params_->cbar2 = cbar2_;
    quatro_params_->estimate_scaling = estimate_scaling_;
    quatro_params_->rotation_max_iterations = teaser_iterations_;
    quatro_params_->rotation_gnc_factor = rotation_gnc_factor_;
    quatro_params_->rotation_cost_threshold = rotation_cost_thre_;
    quatro_params_->rotation_estimation_algorithm = 
        teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::QUATRO;
    quatro_params_->inlier_selection_mode = 
        teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_HEU;
    quatro_ = std::make_unique<teaser::RobustRegistrationSolver>(*quatro_params_);

    if(filtered_map_cloud_->size() > 0) quatro_source_->reserve(filtered_map_cloud_->size());
    for(const auto &point : filtered_map_cloud_->points){
        quatro_source_->push_back({point.x, point.y, point.z});
    }

    quatro_source_feature_ = quatro_fpfh_->computeFPFHFeatures(
        *quatro_source_, source_normal_search_radius_, source_fpfh_search_radius_);
    std::cout << BLUE << "Teaser++ init ..." << RESET;
}

Relocalization::~Relocalization() {}

void Relocalization::setInitialTransformation(){
    initial_transform_.translation() = Eigen::Vector3d(x_, y_, z_);
    Eigen::AngleAxisd roll_rotation(roll_, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_rotation(pitch_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_rotation(yaw_, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw_rotation * pitch_rotation * roll_rotation;
    initial_transform_.linear() = q.toRotationMatrix();
    // result_t_ = initial_transform_;
    // prior_result_t_ = initial_transform_;
}

void Relocalization::loadGlobalMap(
    const std::string& file_name
) {
    // use relative path
    std::string pkg_path = ros::package::getPath("relocalization");
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(pkg_path + "/PCD/" + file_name, *global_map_) == -1){
        ROS_ERROR_STREAM("Couldn't read PCD file: " << file_name.c_str() << " !");
        return;
    }
    std::cout << BLUE << "Loaded global map with " << global_map_->points.size() << " points." << RESET;

    // convert the pointcloud to the map frame
    pcl::transformPointCloud(*global_map_, *global_map_, Eigen::Affine3d(initial_transform_));
}

void Relocalization::imuCallback(
    const sensor_msgs::Imu::ConstPtr &msg
) {
    if(!msg){
        ROS_ERROR("No imu msg input!");
        return;
    }

    imu_buffer_.emplace_back(*msg);
    if(imu_buffer_.size() > 10){
        imu_buffer_.erase(imu_buffer_.begin());
    }
}

int Relocalization::getNearestImuIndex(
    const std::vector<sensor_msgs::Imu> &imu_buffer,
    const ros::Time &stamp
) {
    int imu_index = 0;
    double min_diff = std::numeric_limits<double>::max();
    for(int i = 0; i < imu_buffer.size(); ++i){
        double diff = std::abs(imu_buffer[i ].header.stamp.toSec() - stamp.toSec());
        if(diff < min_diff){
            imu_index = i;
            min_diff = diff;
        }
    }

    return imu_index;
}

void Relocalization::registerPcdCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg
) {
    std::lock_guard<std::mutex> lock(cloud_mutex_); 
    if(!msg){
        // lost_num_++;
        // if(lost_num_ > 20) if_global_localization_ = true;
        ROS_ERROR("No cloud msg input!");
        return;
    }
    
    last_scan_time_ = msg->header.stamp;
    pcl::fromROSMsg(*msg, *registered_scan_);

    /* small gicp */
    source_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>
        (*registered_scan_, registered_leaf_size_);
    small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);
    source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>
       (source_, small_gicp::KdTreeBuilderOMP(num_threads_));

    /* gpu bbs3d */
    // if(if_global_localization_){
    //     filtered_scan_cloud_->clear();
    //     if(min_scan_range_ == 0.0 && max_scan_range_ == 0.0){
    //         *filtered_scan_cloud_ = *registered_scan_;
    //     } else{
    //         for(const auto& point : registered_scan_->points){
    //             auto norm = pcl::euclideanDistance(point, pcl::PointXYZ(0.0f, 0.0f, 0.0f));
    //             if(norm >= min_scan_range_ && norm <= max_scan_range_){
    //                 filtered_scan_cloud_->points.emplace_back(point);
    //             }
    //         }
    //     }    
    //     if(source_leaf_size_ != 0.0f){
    //         pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    //         voxelgrid.setLeafSize(source_leaf_size_, source_leaf_size_, source_leaf_size_);
    //         voxelgrid.setInputCloud(filtered_scan_cloud_);
    //         voxelgrid.filter(*filtered_scan_cloud_);
    //     }
    
    //     auto imu_index = getNearestImuIndex(imu_buffer_, last_scan_time_);
    //     const auto imu_msg = imu_buffer_[imu_index];
    //     const Eigen::Vector3d acc = {imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z};
    
    //     // tf2::Quaternion q(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z,imu_msg.orientation.w);
    //     // tf2::Matrix3x3 mat(q);
    //     // double roll, pitch, yaw;
    //     // mat.getRPY(roll, pitch, yaw);
    //     // current_yaw_ = yaw;
    //     pcl::transformPointCloud(*filtered_scan_cloud_, *filtered_scan_cloud_, pciof::calc_gravity_alignment_matrix(acc.cast<float>()));
    //     pciof::pcl_to_eigen(filtered_scan_cloud_, source_points);
    //     gpu_bbs3d_->set_src_points(source_points);
    // }    
    if(if_global_localization_){
        for(const auto &point : registered_scan_->points){
            quatro_target_->push_back({point.x, point.y, point.z});
        }        
    }
  
}

void Relocalization::initialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg
) {
    if(debug_en_){
        ROS_INFO_STREAM("Received initialpose: " << "x: " << msg->pose.pose.position.x << " "
            << "y; " << msg->pose.pose.position.y << " "
            << "y; " << msg->pose.pose.position.y << " ");
    }

    Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
    map_to_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    map_to_base.linear() = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                               msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                               .toRotationMatrix();
    while(true){
        try{
            auto transform = tf_buffer_->lookupTransform(base_frame_, registered_scan_->header.frame_id, ros::Time(0), ros::Duration(1.0));
            Eigen::Isometry3d base_to_odom = tf2::transformToEigen(transform.transform);
            Eigen::Isometry3d map_to_odom = map_to_base * base_to_odom;
            prior_result_t_ = map_to_odom;
            result_t_ = map_to_odom;
            break;
        }catch (tf2::TransformException & ex){
            ROS_WARN_STREAM("Could not transform initial pose from" << base_frame_.c_str() << " to " 
                            << registered_scan_->header.frame_id.c_str() << ex.what());
            ros::Duration(1.0).sleep();  
        }
    }
}

void Relocalization::runRegistration(
    const ros::WallTimerEvent& event
) {
    std::lock_guard<std::mutex> lock(cloud_mutex_); 
    if(!source_ || !source_tree_){
        return;
    }
    
    if(debug_en_){
        if(pub_prior_pcd_en_){
            pcl::toROSMsg(*global_map_, prior_pcd_msg);    
            prior_pcd_msg.header.frame_id = map_frame_;
            prior_pcd_msg.header.stamp = ros::Time().fromSec(last_scan_time_.toSec());
            pub_pcd_.publish(prior_pcd_msg);
        }
    }

    // if(if_global_localization_){
    //     gpu_bbs3d_->localize();
    //     std::cout << "[gpu_bbs localize] Execution time: " << gpu_bbs3d_->get_elapsed_time() << "[msec] " << std::endl;
    //     std::cout << "[gpu_bbs localize] Score: " << gpu_bbs3d_->get_best_score() << std::endl;
    //     std::cout << "[gpu_bbs localize] pose: \n" << gpu_bbs3d_->get_global_pose() << std::endl;
    //     if (!gpu_bbs3d_->has_localized()) {
    //         if (gpu_bbs3d_->has_timed_out()){
    //             std::cout << "[gpu_bbs failed] Localization timed out." << std::endl;
    //         } else{
    //             std::cout << "[gpu_bbs failed] Score is below the threshold." << std::endl;
    //         }
    //         return;
    //     }
        
    //     result_t_ = Eigen::Isometry3d(gpu_bbs3d_->get_global_pose().cast<double>());
    //     prior_result_t_ = result_t_;
    //     if_global_localization_ = false;
    // }

    if(if_global_localization_){
        auto start_time = std::chrono::high_resolution_clock::now();
        quatro_target_feature_ = quatro_fpfh_->computeFPFHFeatures(
            *quatro_target_, target_normal_search_radius_, target_fpfh_search_radius_);
        teaser::Matcher quatro_matcher;
        auto correspondences = quatro_matcher.calculateCorrespondences(
            *quatro_source_, *quatro_target_, *quatro_source_feature_, *quatro_target_feature_,
            false, true, false, tuple_scale_);
        quatro_->solve(*quatro_source_, *quatro_target_, correspondences);
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> quatro_duration = end_time - start_time;
        
        if(quatro_->getSolution().valid){
            prior_result_t_.translation() = quatro_->getSolution().translation;
            prior_result_t_.linear() = quatro_->getSolution().rotation;
            std::cout << GREEN << "Quatro time: " << quatro_duration.count() << " ms"
                               << "Translation:\n" << prior_result_t_.translation()
                               << "Rotation:\n" << prior_result_t_.linear() << RESET; 
            if_global_localization_ = false;
        } else{
            std::cout << YELLOW << "Teaser+=+ didn't converg!" << RESET;
        }
 
    }
    

    /* small gicp */
    auto align_time_begin = std::chrono::high_resolution_clock::now();
    auto result = gicp_register_->align(*target_, *source_, *target_tree_, prior_result_t_);
    auto align_time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> align_duration = align_time_end - align_time_begin;
    
    if(debug_en_)
        std::cout << GREEN << "[GICP] time: " << align_duration.count() << " ms" << RESET;
    if(!result.converged){
        if(debug_en_){
            ROS_WARN_STREAM("The gicp didn't converge! Time: " << align_duration.count() << " ms");
        }
        return;
    }

    result_t_ = result.T_target_source;
    prior_result_t_ = result.T_target_source;
}

void Relocalization::publishTransform(
    const ros::WallTimerEvent& event
) {
    if(result_t_.matrix().isZero()){
        ROS_WARN_STREAM("No legal transform!");
        return;
    }

    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header.stamp = last_scan_time_ + ros::Duration(transform_tolerance_);
    tf_stamped.header.frame_id = map_frame_;
    tf_stamped.child_frame_id = odom_frame_;

    const Eigen::Vector3d translation = result_t_.translation();
    const Eigen::Quaterniond rotation(result_t_.rotation());

    tf_stamped.transform.translation.x = translation.x();
    tf_stamped.transform.translation.y = translation.y();
    tf_stamped.transform.translation.z = translation.z();
    tf_stamped.transform.rotation.x = rotation.x();
    tf_stamped.transform.rotation.y = rotation.y();
    tf_stamped.transform.rotation.z = rotation.z();
    tf_stamped.transform.rotation.w = rotation.w();
    
    /**
     * @brief publish the transform only when the stamp changes to prevent the error of tf tree
     */
    static ros::Time last_tf_time;
    if(tf_stamped.header.stamp != last_tf_time){
        last_tf_time = tf_stamped.header.stamp;
        tf_broadcaster_->sendTransform(tf_stamped);
    }
}

void Relocalization::dynamicReconfigCallback(
    relocalization::RelocalizationCfgConfig &config, uint32_t level
) {
    std::lock_guard<std::mutex> lock(reconfigure_mutex_);
    if(first_reconfigure_call_)
    {
      first_reconfigure_call_ = false;
      relo_config_ = config;
      return;
    }
  
    if(config.restore_defaults) {
      config = relo_config_;
      //avoid looping
      config.restore_defaults = false;
    }
    debug_en_ = config.debug_en;

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "relocalization_node");
    Relocalization relocalization_node;

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
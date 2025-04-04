#include "std_cout.h"
#include <small_gicp/small_gicp_relocalization.hpp>

SmallGicpRelocalization::SmallGicpRelocalization(): 
    pr_nh_("~"),
    result_t_(Eigen::Isometry3d::Identity()), prior_result_t_(Eigen::Isometry3d::Identity())
{
    pr_nh_.param<bool>("small_gicp/debug", debug_, false);

    pr_nh_.param<int>("small_gicp/num_threads", num_threads_, 4);
    pr_nh_.param<int>("small_gicp/num_neighbors", num_neighbors_, 20);
    pr_nh_.param<float>("small_gicp/global_leaf_size", global_leaf_size_, 0.25);
    pr_nh_.param<float>("small_gicp/registered_leaf_size", registered_leaf_size_, 1.0);
    pr_nh_.param<float>("small_gicp/max_dist_sq", max_dist_sq_, 1.0);
    pr_nh_.param<std::string>("small_gicp/map_frame", map_frame_, "");
    pr_nh_.param<std::string>("small_gicp/odom_frame", odom_frame_, "");
    pr_nh_.param<std::string>("small_gicp/base_frame", base_frame_, "");
    pr_nh_.param<std::string>("small_gicp/lidar_frame", lidar_frame_, "");
    pr_nh_.param<std::string>("small_gicp/pcd_in_topic", pcd_in_topic_, "");
    pr_nh_.param<std::string>("small_gicp/prior_pcd_file", prior_pcd_file_, "");
    pr_nh_.param<std::string>("small_gicp/initialpose_topic", initialpose_topic_, "");

    pr_nh_.param<double>("small_gicp/x", x_, 0.0);
    pr_nh_.param<double>("small_gicp/y", y_, 0.0);
    pr_nh_.param<double>("small_gicp/z", z_, 0.0);
    pr_nh_.param<double>("small_gicp/yaw", yaw_, 0.0);
    pr_nh_.param<double>("small_gicp/roll", roll_, 0.0);
    pr_nh_.param<double>("small_gicp/pitch", pitch_, 0.0);

    pr_nh_.param<double>("small_gicp/registration_frequency", registration_frequency_, 4);
    pr_nh_.param<double>("small_gicp/pose_update_frequency", pose_update_frequency_, 2);

    pr_nh_.param<double>("small_gicp/transform_tolerance", transform_tolerance_, 0.18);

    registered_scan_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    register_ = std::make_shared<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    loadGlobalMap(prior_pcd_file_);

    target_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>
              (*global_map_, global_leaf_size_);
    
    ROS_INFO_STREAM(GREEN << "global map points size is: " << target_->size() << " frame: " << target_->header.frame_id << RESET);

    small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(target_, small_gicp::KdTreeBuilderOMP(num_threads_));

    sub_pcd_ = nh_.subscribe<sensor_msgs::PointCloud2>(pcd_in_topic_.c_str(), 10, &SmallGicpRelocalization::registerPcdCallback, this);

    sub_initialpose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(initialpose_topic_.c_str(), 10, &SmallGicpRelocalization::initialPoseCallback, this);

    pub_pcd_ = nh_.advertise<sensor_msgs::PointCloud2>("/prior_pcd", 10, this);

    register_timer_ = this->createWallTimer(ros::WallDuration(1 / registration_frequency_), &SmallGicpRelocalization::runRegistration, this, false, true);
    transform_timer_ = this->createWallTimer(ros::WallDuration(1 / pose_update_frequency_), &SmallGicpRelocalization::publishTransform, this, false, true);
}

SmallGicpRelocalization::~SmallGicpRelocalization()
{

}
 
//load the globalmap and convert to map

void SmallGicpRelocalization::loadGlobalMap(const std::string& file_name)
{
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1){
        ROS_ERROR_STREAM("Couldn't read PCD file: " << file_name.c_str() << " !");
        return;
    }

    if(debug_)
        ROS_INFO_STREAM("Loaded global map with " << global_map_->points.size() << " points.");

    // convert the pointcloud to the map frame
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() = Eigen::Vector3d(x_, y_, z_);
    Eigen::AngleAxisd roll_rotation(roll_, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_rotation(pitch_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_rotation(yaw_, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw_rotation * pitch_rotation * roll_rotation;
    transform.linear() = q.toRotationMatrix();
    pcl::transformPointCloud(*global_map_, *global_map_, transform);
}

void SmallGicpRelocalization::registerPcdCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    last_scan_time_ = msg->header.stamp;
    
    pcl::toROSMsg(*global_map_, prior_pcd_msg);    
    prior_pcd_msg.header.frame_id = map_frame_;
    prior_pcd_msg.header.stamp = ros::Time().fromSec(msg->header.stamp.toSec());
    pub_pcd_.publish(prior_pcd_msg);
    pcl::fromROSMsg(*msg, *registered_scan_);
    //downsample registered points and convert them into pcl::PointCloud<pcl::PointCovariance>
    source_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*registered_scan_, registered_leaf_size_);

    if(debug_)
        ROS_INFO_STREAM(GREEN << "registered points size is " << source_->size() << " source frame: " << source_->header.frame_id << RESET);
    
    //estimate covariances of points
    small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

    source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(source_, small_gicp::KdTreeBuilderOMP(num_threads_));
}

void SmallGicpRelocalization::runRegistration(const ros::WallTimerEvent& event)
{
    if(!source_ || !source_tree_){
        return;
    }

    register_->reduction.num_threads = num_threads_;
    register_->rejector.max_dist_sq = max_dist_sq_;

    // align_time_begin_ = ros::Time::now().toSec();
    auto result = register_->align(*target_, *source_, *target_tree_, prior_result_t_);
    // align_time_end_ = ros::Time::now().toSec();
    // ROS_INFO_STREAM("The aligned time is " << align_time_end_ - align_time_begin_ << " s.");

    if(!result.converged){
        // ROS_WARN_STREAM("The small gicp didn't converge");
        return;
    }

    result_t_ = result.T_target_source;
    prior_result_t_ = result.T_target_source;
}

void SmallGicpRelocalization::publishTransform(const ros::WallTimerEvent& event)
{
    if(result_t_.matrix().isZero()){
        return;
    }

    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header.stamp = last_scan_time_ + ros::Duration(transform_tolerance_);
    // tf_stamped.header.stamp = ros::Time::now() + ros::Duration(transform_tolerance_);

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
     * @brief 仅当时间戳变化时发布变换,否则终端会弹出很多和时间戳有关的问题，虽然不会影响机器人实际的导航运动
     * 这种问题一般是由于tf树底层发布比顶层快，也可能是因为话题通信中节点发布信息的频率和代码中tf变换发布的频率等不匹配造成的
     * 我最终的解决方案如下，测试时没有太大问题，当然也可能有更好的方案
     */
    static ros::Time last_tf_time;
    if (tf_stamped.header.stamp != last_tf_time){
        last_tf_time = tf_stamped.header.stamp;
        tf_broadcaster_->sendTransform(tf_stamped);
    }
}


void SmallGicpRelocalization::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
    if(debug_){
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
            ROS_WARN_STREAM("Could not transform initial pose from" << base_frame_.c_str() << " to " << registered_scan_->header.frame_id.c_str() << ex.what());
            ros::Duration(1.0).sleep();    
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "small_gicp_relocalization");
    SmallGicpRelocalization small_gicp_relocalization;

    // run with two threads
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}

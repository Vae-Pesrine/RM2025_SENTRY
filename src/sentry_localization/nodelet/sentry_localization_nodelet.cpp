#include <mutex>
#include <memory>
#include <cmath>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

//pcl library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pclomp/ndt_omp.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>

//message
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2_eigen/tf2_eigen.h> 
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <include/std_cout.h>
#include <include/localization/pose_estimator.hpp>
#include <include/localization/delta_estimator.hpp>

#include <sentry_userdefinition/ScanMatchingStatus.h>
#include <sentry_userdefinition/SetGlobalMap.h>
#include <sentry_userdefinition/QueryGlobalLocalization.h>

namespace sentry_localization
{

class SentryLocalizationNodelet : public nodelet::Nodelet
{

public :
    SentryLocalizationNodelet() : tf_buffer(), tf_listener(tf_buffer)  {}
    virtual ~SentryLocalizationNodelet() {}

    void onInit()  override{
        nh = getNodeHandle();
        mt_nh = getMTNodeHandle();
        pr_nh = getPrivateNodeHandle();
        
        InitParams();
        sub_imu = mt_nh.subscribe(pr_nh.param<std::string>("topic_imu", topic_imu), 256, &SentryLocalizationNodelet::imuCb, this); 
        sub_points = mt_nh.subscribe(pr_nh.param<std::string>("topic_lidar", topic_lidar), 5, &SentryLocalizationNodelet::pointsCb, this);
        sub_globalmap = mt_nh.subscribe("/globalmap", 1, &SentryLocalizationNodelet::globalmapCb, this);
        sub_initialpose = mt_nh.subscribe("/initialpose", 1, &SentryLocalizationNodelet::initialposeCb, this);

        pub_pose = nh.advertise<nav_msgs::Odometry>("/state/odom", 5, false);
        pub_alignedcloud = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 5, false);
        pub_status = nh.advertise<sentry_userdefinition::ScanMatchingStatus>("/status", 5, false);

        pr_nh.getParam("global_localization_switch", global_localization_switch);
        if(global_localization_switch){
            std::cout << GREEN << "Waiting for global localization services" << RESET << std::endl;
            ros::service::waitForService("/sentry_userdefinition/set_globalmap");
            ros::service::waitForService("/sentry_userdefinition/query");
            set_globalmap_service = nh.serviceClient<sentry_userdefinition::SetGlobalMap>("/sentry_userdefinition/set_globalmap");
            query_globallocalization_service = nh.serviceClient<sentry_userdefinition::QueryGlobalLocalization>("/sentry_userdefinition/query");

            relocalize_server = nh.advertiseService("/relocalize", &SentryLocalizationNodelet::relocalize, this);
        }
    }

private :
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr createRegistration() const{
        double ndt_resolution = pr_nh.param<double>("ndt_resolution", 0.1);
        pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
        ndt->setTransformationEpsilon(0.01);
        ndt->setResolution(ndt_resolution);
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
        return ndt;
    }

    void InitParams(){
        pr_nh.getParam("odom_frame_id", odom_frame_id);
        pr_nh.getParam("base_frame_id", base_frame_id);

        pr_nh.getParam("voxelGridFilterSize", voxelGridFilterSize);
        boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZI>> voxelGridFilter(new pcl::VoxelGrid<pcl::PointXYZI>());
        voxelGridFilter->setLeafSize(voxelGridFilterSize, voxelGridFilterSize, voxelGridFilterSize);
        downsample_filter = voxelGridFilter;

        std::cout << GREEN << "Create registration NDT for localization" << RESET << std::endl;
        registration = createRegistration();

        std::cout << GREEN << "Create registration method for fallback during relocalization" << RESET << std::endl;
        relocalization_switch = false;
        delta_estimator.reset(new DeltaEstimator(createRegistration()));

        pr_nh.getParam("time_duration", time_duration);
        pr_nh.getParam("origin_pos_x", origin_pos_x);
        pr_nh.getParam("origin_pos_y", origin_pos_y);
        pr_nh.getParam("origin_pos_z", origin_pos_z);
        pr_nh.getParam("origin_ori_x", origin_ori_x);
        pr_nh.getParam("origin_ori_y", origin_ori_y);
        pr_nh.getParam("origin_ori_z", origin_ori_z);
        pr_nh.getParam("origin_ori_w", origin_ori_w);
        pose_estimator.reset(new sentry_localization::PoseEstimator(registration, 
                             Eigen::Vector3f(origin_pos_x, origin_pos_y, origin_pos_z),
                             Eigen::Quaternionf(origin_ori_x, origin_ori_y, origin_ori_z, origin_ori_w),
                             time_duration));
    }

private :
    void imuCb(const sensor_msgs::Imu::ConstPtr imu_msg){
        std::lock_guard<std::mutex> lock(imu_mutex);
        imu_data.push_back(imu_msg);
    }

    void pointsCb(const sensor_msgs::PointCloud2::ConstPtr points_msg){
        if(!globalmap_cloud){
            std::cout << RED << "Globalmap has not been received!" << RESET << std::endl;
            return;
        }

        const auto& stamp = points_msg->header.stamp;           //一帧点云对应的时间戳
        pcl::PointCloud<pcl::PointXYZI>::Ptr livox_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*points_msg, *livox_cloud);

        if(livox_cloud->empty()){
            std::cout << RED << "No livox cloud!" << RESET << std::endl;
            return;        
        }

        //transform pointcloud into base frame id
        std::string tf_error;
        pcl::PointCloud<pcl::PointXYZI>::Ptr livox_cloud_out(new pcl::PointCloud<pcl::PointXYZI>());
        if(this->tf_buffer.canTransform(base_frame_id, livox_cloud->header.frame_id, stamp, ros::Duration(0.1), &tf_error)){
            if(!pcl_ros::transformPointCloud(base_frame_id, *livox_cloud, *livox_cloud_out, this->tf_buffer)){
                std::cout << RED << "Could not transform livox cloud to target frame!" << RESET << std::endl;
                return;
            }
        }
        else{
            std::cout << RED << tf_error << RESET  << std::endl;
            return;
        }


        //体素滤波器降采样
        auto filtered_cloud = downsample(livox_cloud_out);
        last_scan = filtered_cloud;

        if(relocalization_switch){
            delta_estimator->add_frame(filtered_cloud);
        }

        std::lock_guard<std::mutex> pose_estimator_lock(pose_estimator_mutex);
        if(!pose_estimator){
            std::cout << BLUE << "Waiting for initial pose input!" << RESET << std::endl;
            return;
        }

        //imu data prediction
        std::lock_guard<std::mutex> lock(imu_mutex);
        auto imu_iter = imu_data.begin();
        for(imu_iter ; imu_iter != imu_data.end(); ++imu_iter){
            if(stamp < (*imu_iter)->header.stamp){
                break;
            }
            const auto& acc = (*imu_iter)->linear_acceleration;
            const auto& gyro = (*imu_iter)->angular_velocity;
            double acc_sign = invert_acc ? -1.0 : 1.0;
            double gyro_sign = invert_gyro ? -1.0 : 1.0;
            pose_estimator->predict((*imu_iter)->header.stamp, acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
        }
        imu_data.erase(imu_data.begin(), imu_iter);

        //odometry data prediction
        ros::Time last_correction_time = pose_estimator->last_correction_time();
        if(!last_correction_time.isZero()){
            geometry_msgs::TransformStamped odom_delta;
            if(tf_buffer.canTransform(base_frame_id, last_correction_time, base_frame_id, stamp, odom_frame_id, ros::Duration(0.1))){
                odom_delta = tf_buffer.lookupTransform(base_frame_id, last_correction_time, base_frame_id, stamp, odom_frame_id, ros::Duration(0.1));
            }
            else if(tf_buffer.canTransform(base_frame_id, last_correction_time, base_frame_id, ros::Time(0.0), odom_frame_id, ros::Duration(0.0))){
                odom_delta = tf_buffer.lookupTransform(base_frame_id, last_correction_time, base_frame_id, ros::Time(0.0), odom_frame_id, ros::Duration(0.0));
            }

            if(odom_delta.header.stamp.isZero()){
                NODELET_WARN_STREAM("failed to look up transform between" << livox_cloud_out->header.frame_id << "and" << odom_frame_id);
            }
            else{
                Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
                pose_estimator->predict_odom(delta.cast<float>().matrix());
            }
        }

        auto aligned = pose_estimator->correct(stamp, filtered_cloud);

        if(pub_alignedcloud.getNumSubscribers()){
            aligned->header.frame_id = "map";
            aligned->header.stamp = livox_cloud_out->header.stamp;
            pub_alignedcloud.publish(aligned);
        }

        if(pub_status.getNumSubscribers()){
            publish_scanMatchingStatus(points_msg->header, aligned);
        }

        publish_odometry(points_msg->header.stamp, pose_estimator->matrix());
    }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr downsample(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)const{
        if(!downsample_filter){
            return cloud;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        downsample_filter->setInputCloud(cloud);
        downsample_filter->filter(*filtered);
        filtered->header = cloud->header;
        return filtered;
    }

    void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose){
        if(tf_buffer.canTransform(odom_frame_id, base_frame_id, ros::Time(0.0))){
            geometry_msgs::TransformStamped  base_to_map = tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
            base_to_map.header.stamp = stamp;
            base_to_map.header.frame_id = base_frame_id;
            base_to_map.child_frame_id = "map";

            geometry_msgs::TransformStamped odom_to_base = tf_buffer.lookupTransform(odom_frame_id, base_frame_id, ros::Time(0.0), ros::Duration(0.1));

            geometry_msgs::TransformStamped map_wrt_odom;
            tf2::doTransform(base_to_map, base_to_map, odom_to_base);     //out odom_to_map

            tf2::Transform map_to_odom; 
            tf2::fromMsg(base_to_map.transform, map_to_odom);
            map_to_odom = map_to_odom.inverse();
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.transform = tf2::toMsg(map_to_odom);
            odom_trans.header.stamp = stamp;
            odom_trans.header.frame_id = "map";
            odom_trans.child_frame_id = odom_frame_id;

            tf_broadcaster.sendTransform(odom_trans);
        }
        else{
            geometry_msgs::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
            odom_trans.header.stamp = stamp;
            odom_trans.header.frame_id = "map";
            odom_trans.child_frame_id = odom_frame_id;
            tf_broadcaster.sendTransform(odom_trans);
        }

        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "map";
        tf::poseEigenToMsg(Eigen::Isometry3d(pose.cast<double>()), odom.pose.pose);
        odom.child_frame_id = base_frame_id;
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = 0.0;
        pub_pose.publish(odom);
    }
    
    void publish_scanMatchingStatus(const std_msgs::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned){
        sentry_userdefinition::ScanMatchingStatus status;
        status.header = header;
        status.has_converged = registration->hasConverged();
        status.matching_error = 0.0;

        double max_correspondence_dist;
        double max_valid_point_dist;
        pr_nh.getParam("max_correspondence_dist", max_correspondence_dist);
        pr_nh.getParam("max_valid_point_dist", max_valid_point_dist);

        int num_inliers = 0;
        int num_valid_points = 0;
        std::vector<int> k_indices;     //存储查询近邻点索引
        std::vector<float> k_sq_dists;  //存储近邻点对应距离的平方
        for(int i = 0; i < aligned->size(); ++i){
            const auto& pt = aligned->at(i);
            if(pt.getVector3fMap().norm() > max_valid_point_dist){
                continue;
            }
            num_valid_points++;
            registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
            if(k_sq_dists[0] < pow(max_correspondence_dist, 2)){
                status.matching_error += k_sq_dists[0];
                num_inliers++;   //内点
            }
        }

        status.matching_error /= num_inliers;
        status.inlier_fraction = static_cast<float>(num_inliers) / std::max(1, num_valid_points);  //内点比例
        status.relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration->getFinalTransformation().cast<double>())).transform;
        status.prediction_errors.reserve(2);
        status.prediction_labels.reserve(2);

        std::vector<double> errors(6, 0.0);

        if(pose_estimator->wo_prediction_error()){
            status.prediction_labels.push_back(std_msgs::String());
            status.prediction_labels.back().data = "without_pred";
            status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->wo_prediction_error().get().cast<double>())).transform);
        }
        if(pose_estimator->imu_prediction_error()){
            status.prediction_labels.push_back(std_msgs::String());
            status.prediction_labels.back().data = "imu";
            status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->imu_prediction_error().get().cast<double>())).transform);
        }
        if(pose_estimator->odom_prediction_error()){
            status.prediction_labels.push_back(std_msgs::String());
            status.prediction_labels.back().data = "odom";
            status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->odom_prediction_error().get().cast<double>())).transform);
        }
        pub_status.publish(status);
    }    

    void globalmapCb(const sensor_msgs::PointCloud2::ConstPtr& points_msg){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*points_msg, *cloud_temp);
        globalmap_cloud = cloud_temp;
        registration->setInputTarget(globalmap_cloud);

        if(global_localization_switch){
            std::cout << GREEN << "Set globalmap for global localization" << RESET << std::endl;
            sentry_userdefinition::SetGlobalMap srv;
            pcl::toROSMsg(*globalmap_cloud, srv.request.global_map);

            if(!set_globalmap_service.call(srv)){
                std::cout << RED << "Failed to set globalmap!" << RESET << std::endl;
            }
            else{
                std::cout << GREEN << "Set globalmap successfully" << RESET << std::endl;
            }
        }
    }

    bool relocalize(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response){
        if(last_scan == nullptr){
            return false;
        }
        relocalization_switch = true;
        delta_estimator.reset();
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr scan = last_scan;
        sentry_userdefinition::QueryGlobalLocalization srv;
        pcl::toROSMsg(*scan, srv.request.cloud);
        srv.request.max_num_candidates = 1;
        if(!query_globallocalization_service.call(srv) || srv.response.poses.empty()){
            relocalization_switch = false;
            std::cout << RED << "Global localization falied" << RESET << std::endl;
            return false;
        }
        
        const auto& result = srv.response.poses[0];

        Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
        pose.linear() = Eigen::Quaternionf(result.orientation.w, result.orientation.x, result.orientation.y, result.orientation.z).toRotationMatrix();
        pose.translation() = Eigen::Vector3f(result.position.x, result.position.y, result.position.z);
        pose = pose * delta_estimator->estimated_delta();
        
        std::lock_guard<std::mutex> lock(pose_estimator_mutex);
        pose_estimator.reset(new sentry_localization::PoseEstimator(registration, 
                                                                     pose.translation(), 
                                                                     Eigen::Quaternionf(pose.linear()), 
                                                                     time_duration));

        relocalization_switch = false;
        return true;
    }

    void initialposeCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
    NODELET_INFO("initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    pose_estimator.reset(new sentry_localization::PoseEstimator(registration, Eigen::Vector3f(p.x, p.y, p.z), 
                                                             Eigen::Quaternionf(q.w, q.x, q.y, q.z),
                                                             time_duration));
  }



private :
    ros::NodeHandle nh;
    ros::NodeHandle mt_nh;   //multiple thread
    ros::NodeHandle pr_nh;   //private for params
    
    ros::Subscriber sub_imu;
    ros::Subscriber sub_points;
    ros::Subscriber sub_globalmap;
    ros::Subscriber sub_initialpose;

    ros::Publisher pub_pose;
    ros::Publisher pub_alignedcloud;     //配准后的点云
    ros::Publisher pub_status;           //扫描配准状态

    std::string topic_lidar;
    std::string topic_imu;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    std::string odom_frame_id;
    std::string base_frame_id;
    //imu prediction
    double invert_acc;
    double invert_gyro;
    std::mutex imu_mutex;
    std::vector<sensor_msgs::ImuConstPtr> imu_data;

    //globalmap and registration method
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalmap_cloud;
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;
    double voxelGridFilterSize = 0;
    pcl::Filter<pcl::PointXYZI>::Ptr downsample_filter;

    //pose estimator
    std::mutex pose_estimator_mutex;
    std::unique_ptr<sentry_localization::PoseEstimator> pose_estimator;

    //global localization
    bool global_localization_switch;
    std::atomic_bool relocalization_switch;
    std::unique_ptr<sentry_localization::DeltaEstimator> delta_estimator;
    
    double time_duration = 0.5;
    double origin_pos_x, origin_pos_y, origin_pos_z;
    double origin_ori_x, origin_ori_y, origin_ori_z, origin_ori_w;

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr last_scan;

    ros::ServiceServer relocalize_server;

    ros::ServiceClient set_globalmap_service;
    ros::ServiceClient query_globallocalization_service;

};

}

PLUGINLIB_EXPORT_CLASS(sentry_localization::SentryLocalizationNodelet, nodelet::Nodelet)
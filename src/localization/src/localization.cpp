#include <localization.h>

namespace rm_sentry{

sentry_localization::sentry_localization(): pr_nh("~"), tf_listener_(tf_buffer_)
{
    ROS_INFO_STREAM(GREEN << "Sentry localization initialized!" << RESET);

    sub_odom = nh.subscribe("/aft_mapped_to_init", 20, &sentry_localization::OdomCallback, this);

    sub_livox_mid360 = nh.subscribe("/livox_horizon_points", 100, &sentry_localization::CustomMsgToPointcloud, this);

    sub_lidar_points = nh.subscribe("/lidar_points", 100, &sentry_localization::LidarCallback, this);

    pub_lidar_points = nh.advertise<sensor_msgs::PointCloud2>("/lidar_points", 100);
    
    pub_global_map = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 100);

    //ros自动做了pcl::PointCloud<PointT>到sensor_msgs::PointCloud2的转换
    pub_gicp_pointcloud = nh.advertise<pcl::PointCloud<PointT>>("/gicp_pointcloud", 1, this);

    pub_rotate_cloud = nh.advertise<pcl::PointCloud<PointT>>("/rotate_cloud", 1, this);

    pub_removed_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("removed_pointcloud", 10);

    pub_localization_status = nh.advertise<sentry_userdefinition::LocalizationInfo>("/localization_status", 1, this);

    pub_localization = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/localization_match", 1, this);

    pub_relocate_tranform_visuial = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/relocate_visuial_pose", 1, this);

    pub_relocalization_initialpose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/relocalization_initialpose", 1, this);

    cloud_scan = boost::shared_ptr<pcl::PointCloud<PointT>>(new pcl::PointCloud<PointT>());
    cloud_map = boost::shared_ptr<pcl::PointCloud<PointT>>(new pcl::PointCloud<PointT>());

    InitParams();
    PcdmapToROS();
}

sentry_localization::~sentry_localization()
{
}

void sentry_localization::InitParams()
{
    pr_nh.param<std::string>("pcd_map", pcd_map, "");
    pr_nh.param<double>("downsample_resolution_globalmap", downsample_resolution_globalmap, 0.1);

    pr_nh.param<bool>("if_debug", if_debug_, "true");

    pr_nh.param<std::string>("odom_frame", odom_frame_, "odom");
    pr_nh.param<std::string>("base_frame", base_frame_, "base_link");
    pr_nh.param<std::string>("map_frame", odom_frame_, "map");

    pr_nh.param<double>("variance_x", Variance_X, 0.2);
    pr_nh.param<double>("variance_y", Variance_Y, 0.2);
    pr_nh.param<double>("variance_yaw", Variance_Yaw, 0.1);

    pr_nh.param<double>("score_thre", SCORE_THRE, 0.13);
    pr_nh.param<double>("dist_thre", DIST_THRE, 0.01);
    pr_nh.param<double>("angle_thre", ANGLE_THRE, 0.01);
    pr_nh.param<double>("angle_speed_thre", ANGLE_SPEED_THRE, 10);
    pr_nh.param<double>("angle_upper_thre", ANGLE_UPPER_THRE, 10);
    pr_nh.param<int>("point_num_thre", POINT_NUM_THRE, 180);
    pr_nh.param<int>("localization_iterations_thre", LOCALIZATION_ITERATIONS_THRE, 5);

    pr_nh.param<double>("voxelremoval_resolution", VoxelRemoval_Resolution, 0.1);
    pr_nh.param<double>("obstacle_distance_thre", OBSTACLE_DISTANCE_THRE, 1.0);
    
    pr_nh.param<float>("relocalization_weight_score", Relocation_Weight_Score_, 1);
    pr_nh.param<float>("relocalization_weight_distance", Relocation_Weight_Distance_, 1);
    pr_nh.param<float>("relocalization_weight_yaw", Relocation_Weight_Yaw_, 1);
    pr_nh.param<int>("relocalization_iterations_thre", RELOCALIZATION_ITERATIONS_THRE, 80);
    pr_nh.param<int>("loss_num_thre", LOSS_NUM_THRE, 0);
    pr_nh.param<float>("relocalization_score_thre", RELOCALIZATION_SCORE_THRE, 0.8);

    pr_nh.param<int>("odom_queue_length", odom_queue_length, 300);
}

void sentry_localization::OdomCallback(const nav_msgs::Odometry::ConstPtr &odometry_msg)
{
    std::lock_guard<std::mutex> lock(odom_lock);
    odom_initialized = true;
    odom_queue.push_back(*odometry_msg);
    if (odom_queue.size() > odom_queue_length)    //弹出超过长度的数据
    {
        odom_queue.pop_front();
    }

    odom_initialized_ = true;
}

void sentry_localization::LidarCallback(const sensor_msgs::PointCloud2::ConstPtr &lidar_msgs)
{
    time_start_ = std::chrono::steady_clock::now();

    if(!odom_initialized_ || !map_initialized_){
        return;
    }

    if(!scan_initialized_){
        if(if_debug_){
            ROS_INFO_STREAM("Lidarcallbacl initialized!");
        }

        scan_time_ = lidar_msgs->header.stamp.toSec();

        map_to_base_ = Eigen::Isometry3d::Identity();
        // if (!GetTransform(map_to_base_, map_frame_, base_frame_, lidar_msgs->header.stamp))
        // {
        //     ROS_WARN("Did not get base pose at now");
        //     return;
        // }

        scan_initialized_ = true;
    }
    else{
        if(if_debug_){
            ROS_INFO_STREAM("Lidarcallback initialized!");
        }

        last_match_time_ = match_time_;         //保存上一次匹配时间和结果
        last_match_result_ = match_result_;
        scan_time_ = lidar_msgs->header.stamp.toSec();   //当前帧
        
        if(ros::Time::now().toSec() - scan_time_ > AGE_THRE){
            ROS_WARN_STREAM("Timeout for lidar scan");
            scan_initialized_ = false;
        }
        
        Eigen::Isometry3d baselast_to_basenow = Eigen::Isometry3d::Identity();
        if(!GetOdomTransform(baselast_to_basenow, last_match_time_, scan_time_)){
            ROS_WARN_STREAM("Didn't get best pose on odom now");
            return;
        }
        map_to_base_ = baselast_to_basenow * last_match_result_;
    }

    pcl::fromROSMsg(*lidar_msgs, *cloud_scan);
    pcl::fromROSMsg(pcdmap_cloud_ros, *cloud_map);

    if(need_relocalization_){
        if(RelocalizationWithGICP(match_result_, cloud_scan, cloud_map, map_to_base_)){
            need_relocalization_ = false;
            relocalization_result_ = true;
        }
        need_relocalization_ = false;
        relocalization_result_ = false;
        return;
    } 
    else{
        PointCloudVoxelGridRemoval(cloud_scan, VoxelRemoval_Resolution);
        PointCloudVoxelGridRemoval(cloud_map, VoxelRemoval_Resolution);

        PointCloudObstacleRemoval(cloud_map, cloud_scan, OBSTACLE_DISTANCE_THRE);

        match_result_ = Eigen::Isometry3d::Identity();  
        if(!ScanMatchWithGICP(match_result_, cloud_scan, cloud_map)){
            scan_initialized_ = false;
            return;
        }
    }

    match_result_ = match_result_ * map_to_base_;  // 将结果转换到map坐标系

    Eigen::Quaterniond q = Eigen::Quaterniond(match_result_.rotation());
    std_msgs::Header header;
    header.stamp = lidar_msgs->header.stamp;;
    header.frame_id = "map";
    localization_match.header = header;
    localization_match.pose.pose.orientation.x = q.x();
    localization_match.pose.pose.orientation.y = q.y();
    localization_match.pose.pose.orientation.z = q.z();
    localization_match.pose.pose.orientation.w = q.w();
    // localization_match.pose.orientation = q;
    localization_match.pose.pose.position.x = match_result_.translation()(0);
    localization_match.pose.pose.position.y = match_result_.translation()(1);
    localization_match.pose.pose.position.z = match_result_.translation()(2);
    //x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
    localization_match.pose.covariance = {Variance_X, 0, 0, 0, 0, 0,  
                                    0, Variance_Y, 0, 0, 0, 0,  
                                    0, 0, 1e-9, 0, 0, 0,  
                                    0, 0, 0, 1e-9, 0, 0, 
                                    0, 0, 0, 0, 1e-9, 0,  
                                    0, 0, 0, 0, 0, Variance_Yaw};
    pub_localization.publish(localization_match);

    time_end_ = std::chrono::steady_clock::now();
    time_duration_ = std::chrono::duration_cast<std::chrono::duration<double>>(time_start_ - time_end_);
    if(if_debug_){
        ROS_INFO_STREAM(GREEN << "The function LidarCallback use time: " << time_duration_.count() << " s" << RESET);
    }

    time_duration_ = std::chrono::duration_cast<std::chrono::duration<double>>(time_end_ - time_lastend_);
    if(if_debug_){
        ROS_INFO_STREAM(GREEN << "The function LidarCallback frequency: " << 1.0/time_duration_.count() << "HZ" << RESET);
    }
    time_lastend_ = time_end_;
}

bool sentry_localization::ScanMatchWithGICP(Eigen::Isometry3d &trans, pcl::PointCloud<PointT>::Ptr &cloud_scan_msg, pcl::PointCloud<PointT>::Ptr &cloud_map_msg)
{
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp_;
    gicp_.setTransformationEpsilon(1e-6);
    gicp_.setEuclideanFitnessEpsilon(1e-6);
    gicp_.setMaximumIterations(LOCALIZATION_ITERATIONS_THRE);
    gicp_.setInputSource(cloud_scan_msg);
    gicp_.setInputTarget(cloud_map_msg);

    pcl::PointCloud<PointT> gicp_result;
    gicp_.align(gicp_result);

    if(gicp_.hasConverged() == false){
        ROS_ERROR_STREAM("The match result not converged!");
        return false;
    }

    Eigen::Affine3f transform;
    transform = gicp_.getFinalTransformation();

    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transform, x, y, z, roll, pitch, yaw);
    double tranDist = sqrt(x * x + y * y);
    double angleDist = abs(yaw);

    sentry_userdefinition::LocalizationInfo localization_status_msg;
    localization_status_msg.if_relocation = false;
    localization_status_msg.point_cloud_quantity = cloud_scan_msg->points.size();
    localization_status_msg.tranDist = tranDist;
    localization_status_msg.angleDist = angleDist;
    localization_status_msg.angle_apeed = abs(odom_queue.back().twist.twist.angular.z);
    localization_status_msg.score = gicp_.getFitnessScore();

    //变换不满足阈值
    if(tranDist < DIST_THRE && angleDist < ANGLE_THRE || cloud_scan_msg->points.size() < POINT_NUM_THRE ){
        if(if_debug_){
            ROS_INFO_STREAM(YELLOW << "Distance or point_Quantity out of threshold" << RESET);
            ROS_INFO_STREAM(YELLOW << "tranDist:"<< tranDist << "angleDist" << angleDist << 
                            "score:" << gicp_.getFitnessScore() << "angle speed" << odom_queue.back().twist.twist.angular.z << RESET);
        }
        localization_status_msg.if_match_success = false;
        pub_localization_status.publish(localization_status_msg);
        return false;
    }

    //结果不满足阈值
    if(angleDist > ANGLE_UPPER_THRE || gicp_.getFitnessScore() > SCORE_THRE || abs(odom_queue.back().twist.twist.angular.z) > ANGLE_SPEED_THRE)
    {
        if(if_debug_){
            ROS_INFO_STREAM(YELLOW << "Result out of threshold" << RESET);
            ROS_INFO_STREAM(YELLOW << "tranDist: "<< tranDist << "angleDist: " << angleDist << 
                            "score: " << gicp_.getFitnessScore() << "angle_speed: " << odom_queue.back().twist.twist.angular.z << RESET);
        }

        localization_loss_num += 1;
        if (localization_loss_num > LOSS_NUM_THRE){
            need_relocalization_ = true;
        }
        if(if_debug_){
            ROS_INFO_STREAM(BLUE << "localization_loss_num: " << localization_loss_num << RESET);
        }
        
        localization_status_msg.if_match_success = false;
        pub_localization_status.publish(localization_status_msg);
        return false;
    }

    localization_loss_num = 0;

    trans.matrix() = transform.matrix().cast<double>();

    localization_status_msg.if_match_success = true;
    pub_localization_status.publish(localization_status_msg);
    pub_gicp_pointcloud.publish(gicp_result);
    return true;
}

bool sentry_localization::RelocalizationWithGICP(Eigen::Isometry3d &trans ,pcl::PointCloud<PointT>::Ptr &cloud_scan_msg, pcl::PointCloud<PointT>::Ptr &cloud_map_msg, const Eigen::Isometry3d &robot_pose)
{
    // 设置不同的旋转角度生成不同的初始变换矩阵
    std::vector<Eigen::Matrix4f> initial_transforms;
    std::vector<float> angles;
    for (float angle = 0.0; angle < 360.0; angle += 10.0) {
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(angle * M_PI / 180.0, Eigen::Vector3f::UnitZ()));
        
        initial_transforms.push_back(transform.matrix());
        angles.push_back(angle * M_PI / 180.0);
    }
    
    std::vector<float> fitness_scores;
    std::vector<Eigen::Affine3f> Transformation_sources;
    std::vector<pcl::PointCloud<PointT>::Ptr> pointcloud_sources;

    for (int i = 0; i < initial_transforms.size(); i++) {

        //  Matrix4f to Affine3f 类型
        Eigen::Affine3f initial_affine(initial_transforms[i]);
        pcl::PointCloud<PointT>::Ptr rotate_scan_cloud(new pcl::PointCloud<PointT>(*cloud_scan_msg));
        RotatePointCloud(rotate_scan_cloud, initial_affine, robot_pose.cast<float>());

        if(if_debug_){
            pub_rotate_cloud.publish(rotate_scan_cloud);
        }

        // 对每个初始变换矩阵进行ICP匹配，并计算匹配分数
        pcl::IterativeClosestPoint<PointT, PointT> gicp_;
        gicp_.setInputSource(rotate_scan_cloud);
        gicp_.setInputTarget(cloud_map_msg);
        gicp_.setMaxCorrespondenceDistance(15);
        gicp_.setMaximumIterations(RELOCALIZATION_ITERATIONS_THRE);
        gicp_.setTransformationEpsilon(1e-8);
        gicp_.setEuclideanFitnessEpsilon(0.01);

        //收敛的点云和变换
        pcl::PointCloud<PointT>::Ptr pointcloud_result (new pcl::PointCloud<PointT>);
        gicp_.align(*pointcloud_result);

        Eigen::Affine3f transfrom = Eigen::Affine3f::Identity();
        transfrom = gicp_.getFinalTransformation();

        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transfrom, x, y, z, roll, pitch, yaw);

        double tranDist = sqrt(x * x + y * y);
        double angleDist = abs(yaw);
        float fitness_score = Relocation_Weight_Score_ * gicp_.getFitnessScore() + Relocation_Weight_Distance_ * tranDist + Relocation_Weight_Yaw_ * angleDist;
        
        fitness_scores.push_back(fitness_score);
        Transformation_sources.push_back(transfrom);
        pointcloud_sources.push_back(pointcloud_result);
        //可视化
        ROS_INFO_STREAM(PURPLE_RED << "--------RESULT--------" << RESET);
        ROS_INFO_STREAM(PURPLE_RED << "x: " << x << " y: " << y << RESET);
        ROS_INFO_STREAM(PURPLE_RED << "tranDist: " << tranDist << " angleDist: " << angleDist << " score: " << gicp_.getFitnessScore() << RESET);
        ROS_INFO_STREAM(PURPLE_RED << "fitness_score: " << fitness_score << RESET);
        
        pub_gicp_pointcloud.publish(pointcloud_result);

        // 将initial_transform从Matrix4f转换为Isometry3d
        Eigen::Isometry3d initial_transform = Eigen::Isometry3d::Identity();
        initial_transform.linear() = initial_transforms[i].cast<double>().matrix().block<3, 3>(0, 0);
        initial_transform.translation() = initial_transforms[i].cast<double>().matrix().block<3, 1>(0, 3);

        Eigen::Isometry3d transfrom_gicp = Eigen::Isometry3d::Identity();
        transfrom_gicp.matrix() = transfrom.matrix().cast<double>();

        transfrom_gicp = robot_pose.inverse() *  transfrom_gicp * robot_pose;

        //将icp结果从Matrix4f类型转换为Isometry3d类型，并点乘初始变换
        Eigen::Isometry3d relocate_tranform_visuial_ = Eigen::Isometry3d::Identity();     //重定位过程可视化坐标
        relocate_tranform_visuial_ = robot_pose * transfrom_gicp * initial_transform; 

        geometry_msgs::PoseWithCovarianceStamped relocation_match_visual;    //重定位可视化结果
        relocation_match_visual = Isometry3d_to_PoseWithCovarianceStamped(relocate_tranform_visuial_);
        pub_relocate_tranform_visuial.publish(relocation_match_visual);
    }

    // 找到分数最高的匹配结果
    // float best_score = std::accumulate(fitness_scores.begin(), fitness_scores.end(), std::initializer_list<float>{*fitness_scores}, std::max<floar>());
    int best_index = 0;
    float best_score = fitness_scores[0];
    for (int i = 1; i < fitness_scores.size(); ++i) {
        if (fitness_scores[i] < best_score) {
            best_index = i;
            best_score = fitness_scores[i];
        }
    }

    // 将initial_transform从Matrix4f转换为Isometry3d
    Eigen::Isometry3d initial_transform = Eigen::Isometry3d::Identity();                                                                                                                                                                           
    initial_transform.linear() = initial_transforms[best_index].cast<double>().matrix().block<3, 3>(0, 0);
    initial_transform.translation() = initial_transforms[best_index].cast<double>().matrix().block<3, 1>(0, 3);

    // 将gicp结果从Matrix4f转换为Isometry3d，并转换到robot_pose
    Eigen::Isometry3d transfrom_gicp = Eigen::Isometry3d::Identity();
    transfrom_gicp.matrix() = Transformation_sources[best_index].matrix().cast<double>();
    transfrom_gicp = robot_pose.inverse() *  transfrom_gicp * robot_pose;
    //将gicp结果从Matrix4f类型转换为Isometry3d类型，并点乘初始变换

    trans = robot_pose * transfrom_gicp * initial_transform;

    geometry_msgs::PoseWithCovarianceStamped relocation_initialpose;    //重定位结果
    relocation_initialpose = Isometry3d_to_PoseWithCovarianceStamped(trans);
    pub_relocalization_initialpose.publish(relocation_initialpose);

    pub_gicp_pointcloud.publish(pointcloud_sources[best_index]);   //发布匹配结果

    sentry_userdefinition::LocalizationInfo location_info_msg;
    location_info_msg.if_relocation = true;
    location_info_msg.point_cloud_quantity = cloud_scan_msg->points.size();
    location_info_msg.tranDist = 0;
    location_info_msg.angleDist = 0;
    location_info_msg.angle_apeed = abs(odom_queue.back().twist.twist.angular.z);
    location_info_msg.score = best_score;

    if(if_debug_){
        ROS_INFO_STREAM(PURPLE_RED << "Best fitness score: " << best_score << RESET);
        ROS_INFO_STREAM(PURPLE_RED << "Besr index score: " << best_index << RESET);
        ROS_INFO_STREAM(PURPLE_RED << "Best transformation matrix: " << Transformation_sources[best_index].matrix() << RESET);
    }

    if(best_score > RELOCALIZATION_SCORE_THRE){
        ROS_INFO_STREAM(RED << "Failed to relocalize, you are unlucky" << RESET);
        location_info_msg.if_match_success = false;
        pub_localization_status.publish(location_info_msg);    //定位信息发布
        return false;
    }

    location_info_msg.if_match_success = true;
    pub_localization_status.publish(location_info_msg);    //定位信息发布

    return true;
}

void sentry_localization::RotatePointCloud(pcl::PointCloud<PointT>::Ptr &cloud_msg, const Eigen::Affine3f &rotation, const Eigen::Affine3f &robot_pose)
{
    //将点云转换到原点
    pcl::transformPointCloud(*cloud_msg, *cloud_msg, robot_pose.inverse());
    //绕原点进行旋转
    pcl::transformPointCloud(*cloud_msg, *cloud_msg, rotation);
    //将点云平移回去
    pcl::transformPointCloud(*cloud_msg, *cloud_msg, robot_pose);
}

//从odom中读取start time 到 end time时间段的坐标转换
bool sentry_localization::GetOdomTransform(Eigen::Isometry3d &trans, double start_time, double end_time)
{
    ros::Rate loop_rate(100);
    while(odom_queue.back().header.stamp.toSec() < end_time && odom_queue.front().header.stamp.toSec() < start_time && ros::ok()){
        ROS_WARN_STREAM("Waiting for odomery data");
        loop_rate.sleep();
        ROS_INFO_STREAM(BLUE << "Time of back" << odom_queue.back().header.stamp.toSec() << RESET);
        ROS_INFO_STREAM(BLUE << "Time of end time" << end_time << RESET);
    }
    if(odom_queue.empty() || odom_queue.front().header.stamp.toSec() > start_time){
        ROS_WARN_STREAM("Start time out of odometry data");
        return false;
    }

    ROS_INFO_STREAM(BLUE << "Time of front: " << odom_queue.front().header.stamp.toSec() <<RESET);
    ROS_INFO_STREAM(BLUE << "Time of back: " << odom_queue.back().header.stamp.toSec() << RESET);
    ROS_INFO_STREAM(BLUE << "Time of start: " << start_time << RESET);
    ROS_INFO_STREAM(BLUE << "Time of end: " << end_time << RESET);

    //get the most simaliar odometry data queue to the required lidar data
    double current_odom_time;
    nav_msgs::Odometry start_odom_msg, end_odom_msg;
    for(int i = 0; i < (int)odom_queue.size(); ++i){
        current_odom_time = odom_queue[i].header.stamp.toSec();
        if(current_odom_time < start_time){
            start_odom_msg = odom_queue[i];
            continue;
        }

        if(current_odom_time <= end_time){
            end_odom_msg = odom_queue[i];
        }
        else
            break;
    }

    double start_odom_time, end_odom_time;
    start_odom_time = start_odom_msg.header.stamp.toSec();
    end_odom_time = end_odom_msg.header.stamp.toSec();

    //odom开始时的位移与旋转
    double start_roll, start_pitch, start_yaw, start_x, start_y, start_z;
    Eigen::Isometry3d trans_begin = Eigen::Isometry3d::Identity();

    tf2::Quaternion quat_start(start_odom_msg.pose.pose.orientation.x, start_odom_msg.pose.pose.orientation.y,
                               start_odom_msg.pose.pose.orientation.z, start_odom_msg.pose.pose.orientation.w);
    start_x = start_odom_msg.pose.pose.position.x;
    start_y = start_odom_msg.pose.pose.position.y;
    start_z = start_odom_msg.pose.pose.position.z;
    tf2::Matrix3x3(quat_start).getRPY(start_roll, start_pitch, start_yaw);

    Eigen::Matrix3d start_rotation;
    start_rotation = Eigen::AngleAxisd(start_yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(start_pitch, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(start_roll, Eigen::Vector3d::UnitX());
    trans_begin.rotate(start_rotation);
    trans_begin.pretranslate(Eigen::Vector3d(start_x, start_y, start_z));

    //odom结束时的位移与旋转
    double end_roll, end_pitch, end_yaw, end_x, end_y, end_z;
    Eigen::Isometry3d trans_end = Eigen::Isometry3d::Identity();

    tf2::Quaternion quat_end(end_odom_msg.pose.pose.orientation.x, end_odom_msg.pose.pose.orientation.y,
                               end_odom_msg.pose.pose.orientation.z, end_odom_msg.pose.pose.orientation.w);
    end_x = end_odom_msg.pose.pose.position.x;
    end_y = end_odom_msg.pose.pose.position.y;
    end_z = end_odom_msg.pose.pose.position.z;
    tf2::Matrix3x3(quat_end).getRPY(end_roll, end_pitch, end_yaw);

    Eigen::Matrix3d end_rotation; 
    end_rotation = Eigen::AngleAxisd(end_yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(end_pitch, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(end_roll, Eigen::Vector3d::UnitX());
    trans_end.rotate(end_rotation);
    trans_end.pretranslate(Eigen::Vector3d(end_x, end_y, end_z));

    trans = trans_end * trans_begin.inverse();
    ROS_INFO_STREAM(GREEN << "Trans matrix is : " << trans.matrix() << RESET);
    return true;
}

//map到baselink两个时刻之间的变换
bool sentry_localization::Get2TimeTransform(Eigen::Isometry3d &trans)
{
    Eigen::Isometry3d transBegin = Eigen::Isometry3d::Identity();
    transBegin = map_to_base_;

    Eigen::Isometry3d transEnd = Eigen::Isometry3d::Identity();
    // transEnd = Eigen::Isometry3d::Identity();     //map到base的欧式变换矩阵4x4
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        transformStamped = tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("DIDNT GET TRANSFORM ON Get2TimeTransform");
        ros::Duration(1.0).sleep();
        return false;
    }
    tf2::Quaternion quat(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

    double roll, pitch, yaw;   //定义存储r\p\y的容器
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    // std::cout << "trans roll, pitch, yaw =  \n" << roll << "  " << pitch << "  " << yaw << std::endl;

    Eigen::Matrix3d point_rotation;
    point_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    transEnd.rotate(point_rotation);
    transEnd.pretranslate(Eigen::Vector3d(transformStamped.transform.translation.x,
                                transformStamped.transform.translation.y,
                                transformStamped.transform.translation.z));

    // 计算得到这段时间的坐标变换
    trans = transEnd *  transBegin.inverse();
    // std::cout << "trans matrix =  \n" << trans.matrix() << std::endl;
    return true;

}


bool sentry_localization::GetTransform(Eigen::Isometry3d &trans , const std::string parent_frame, const std::string child_frame, const ros::Time stamp)
{
    bool gotTransform = false;
    trans = Eigen::Isometry3d::Identity();     //map到base的欧式变换矩阵4x4
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        gotTransform = true;
        transformStamped = tf_buffer_.lookupTransform(parent_frame, child_frame, stamp, ros::Duration(1.0));
        // std::cout << "input rostime of Transform:" << stamp <<std::endl;
        // std::cout << "output rostime of Transform:" << transformStamped.header.stamp <<std::endl;
    }
    catch (tf2::TransformException &ex)
    {
        gotTransform = false;
        ROS_WARN("DIDNT GET TRANSFORM %s %s IN B", parent_frame.c_str(), child_frame.c_str());
        ros::Duration(1.0).sleep();
        return false;
    }

    tf2::Quaternion quat(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

    double roll, pitch, yaw;   //定义存储r\p\y的容器
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    // std::cout << "trans roll, pitch, yaw =  \n" << roll << "  " << pitch << "  " << yaw << std::endl;

    Eigen::Matrix3d point_rotation;
    point_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    // std::cout << "point_rotation = " << point_rotation <<std::endl;

    trans.rotate(point_rotation);
    trans.pretranslate(Eigen::Vector3d(transformStamped.transform.translation.x,
                                transformStamped.transform.translation.y,
                                transformStamped.transform.translation.z));


    return gotTransform;
}

//障碍点剔除
void sentry_localization::PointCloudObstacleRemoval(pcl::PointCloud<PointT>::Ptr &cloud_map_msg, pcl::PointCloud<PointT>::Ptr &cloud_msg, double DISTANCE_THRESHOLD)
{
    pcl::PointCloud<PointT>::Ptr cloud_removed(new pcl::PointCloud<PointT>);

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud_map_msg);
    int K = 1;
    
    for(int i = 0; i < cloud_msg->points.size(); ++i){
        PointT search_point = cloud_msg->points[i];

        std::vector<int>pointIdxNKNSearch(K);
        std::vector<float>pointNKNSquareDistance(K);

        if(kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch, pointNKNSquareDistance) > 0){
            if(pointNKNSquareDistance[0] > DISTANCE_THRESHOLD){
                cloud_msg->erase(cloud_msg->begin() + i);
                cloud_removed->push_back(search_point);
            } else{
                ++i;
            }
        }
    }

    cloud_removed->width = cloud_removed->points.size();
    cloud_removed->height = 1;
    cloud_removed->is_dense = false;  //包括离群点

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    cloud_removed->header = pcl_conversions::toPCL(header);
    pub_removed_pointcloud.publish(cloud_removed);
}

//离群点滤波
void sentry_localization::PointCloudOutlierRemoval(pcl::PointCloud<PointT>::Ptr &cloud_msg)
{
    pcl::StatisticalOutlierRemoval<PointT> sor_OutRemove;
    sor_OutRemove.setInputCloud(cloud_msg);
    sor_OutRemove.setMeanK(30);    //查询近邻点数
    sor_OutRemove.setStddevMulThresh(1.0);   //如果一个点的距离超过平均距离一个标准差以上则为离群点
    sor_OutRemove.filter(*cloud_msg);
}

//体素滤波
void sentry_localization::PointCloudVoxelGridRemoval(pcl::PointCloud<PointT>::Ptr &cloud_msg, double resolution)
{
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud_msg);
    voxel_grid.setLeafSize(resolution, resolution, resolution);
    voxel_grid.filter(*cloud_msg);
    if(if_debug_){
        ROS_INFO_STREAM(BLUE << "Voxel grid removal: " << cloud_msg->points.size() << RESET);
    }
}

geometry_msgs::PoseWithCovarianceStamped sentry_localization::Isometry3d_to_PoseWithCovarianceStamped(const Eigen::Isometry3d& iso)
{
    // 创建一个PoseWithCovarianceStamped类型的消息
    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    // 填充消息头
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";

    //旋转矩阵转四元数
    Eigen::Quaterniond q = Eigen::Quaterniond(iso.rotation());

    // 填充消息头
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    pose_msg.header = header;
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();
    // pose_msg.pose.orientation = q;
    pose_msg.pose.pose.position.x = iso.translation()(0);
    pose_msg.pose.pose.position.y = iso.translation()(1);
    pose_msg.pose.pose.position.z = iso.translation()(2);
    //x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis

    return pose_msg;
}

//雷达扫描的点云数据
void sentry_localization::CustomMsgToPointcloud(const livox_ros_driver2::CustomMsg::ConstPtr &livox_msg_in)
{
    livox_data.push_back(livox_msg_in);
    if(livox_data.size() < 1){
        return;
    }

    pcl::PointCloud<pcl::PointXYZINormal> pcl_in;
    for(std::size_t i = 0; i < livox_data.size(); ++i){
        auto livox_msg = livox_data[i];
        auto time_end = livox_msg->points.back().offset_time;
        
        for(int j = 0; j < livox_msg->point_num; ++j){
            pcl::PointXYZINormal pt;
            pt.x = livox_msg->points[j].x - 0.175;
            pt.y = livox_msg->points[j].y;
            pt.z = livox_msg->points[j].z - 0.46;
            pt.intensity = livox_msg->points[j].line + livox_msg->points[j].reflectivity / 10000.0;
            pt.curvature = static_cast<float>(livox_msg->points[j].offset_time / (float)time_end) * 0.1;
            pcl_in.push_back(pt); 
        }
    }

    unsigned long timebase_ns = livox_data[0]->timebase;
    sensor_msgs::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg(pcl_in, pcl_ros_msg);
    pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
    pcl_ros_msg.header.frame_id = livox_msg_in->header.frame_id;
    pub_lidar_points.publish(pcl_ros_msg);
    pub_global_map.publish(pcdmap_cloud_ros);

    livox_data.clear();
}

//pcd地图转点云数据
void sentry_localization::PcdmapToROS()
{
    pcdmap_cloud.reset(new pcl::PointCloud<PointT>());
    ROS_INFO_STREAM(BLUE << "Reading pcd file: " << pcd_map << RESET);
    if(pcl::io::loadPCDFile(pcd_map, *pcdmap_cloud) == -1){
        return;
    }

    pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>());
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelGridFilter(new pcl::VoxelGrid<PointT>());
    voxelGridFilter->setLeafSize(downsample_resolution_globalmap, downsample_resolution_globalmap, downsample_resolution_globalmap);
    voxelGridFilter->setInputCloud(pcdmap_cloud);
    voxelGridFilter->filter(*filteredCloud);

    pcl::toROSMsg(*filteredCloud, pcdmap_cloud_ros);
    pcdmap_cloud_ros.header.frame_id = "map";

    map_initialized_ = true;

    filteredCloud->clear();
}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sentry_localization_node"); // 节点的名字
    rm_sentry::sentry_localization sentry_localization_node;

    ros::MultiThreadedSpinner spinner(8);
    spinner.spin();
    return 0;
}
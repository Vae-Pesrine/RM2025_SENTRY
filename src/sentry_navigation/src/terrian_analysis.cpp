#include "terrian_analysis.hpp"

TerrainAnalysis::TerrainAnalysis():
    pr_nh_("~")
{
    pr_nh_.param<std::string>("terrian_analysis/odometry_topic", odometry_topic_, "");
    pr_nh_.param<std::string>("terrian_analysis/cloud_topic", cloud_topic_, "");
    pr_nh_.param<std::string>("terrian_analysis/sensor_frame", sensor_frame_, "");
    pr_nh_.param<double>("terrian_analysis/scan_voxel_size", scan_voxel_size_, 0.1);
    pr_nh_.param<double>("terrian_analysis/decay_time", decay_time_, 0.1);
    pr_nh_.param<double>("terrian_analysis/no_decay_dis", no_decay_dis_, 2.0);
    pr_nh_.param<double>("terrian_analysis/clearing_dis", clearing_dis_, 1.0);
    pr_nh_.param<bool>("terrian_analysis/use_sorting", use_sorting_, true);
    pr_nh_.param<double>("terrian_analysis/quantile_z", quantile_z_, 0.1);
    pr_nh_.param<bool>("terrian_analysis/consider_drop", consider_drop_, true);
    pr_nh_.param<double>("terrian_analysis/max_ground_lift", max_ground_lift_, 0.1);
    pr_nh_.param<bool>("terrian_analysis/limit_ground_lift", limit_ground_lift_, false);
    pr_nh_.param<bool>("terrian_analysis/clear_dy_obs", clear_dy_obs_, false);
    pr_nh_.param<double>("terrian_analysis/min_dy_obs_dis", min_dy_obs_dis_, 0.1);
    pr_nh_.param<double>("terrian_analysis/min_dy_obs_angle", min_dy_obs_angle_, 0.1);
    pr_nh_.param<double>("terrian_analysis/min_dy_obs_relZ", min_dy_obs_rel_z_);
    pr_nh_.param<double>("terrian_analysis/abs_dy_obs_relZ_thre", abs_dy_obs_rel_z_thre_);
    pr_nh_.param<double>("terrian_analysis/min_dy_obs_vfoc", min_dy_obs_vfov_);
    pr_nh_.param<double>("terrian_analysis/max_dy_obs_vfoc", max_dy_obs_vfov_);
    pr_nh_.param<int>("terrian_analysis/min_dy_obs_point_num", min_dy_obs_point_num_);
    pr_nh_.param<bool>("terrian_analysis/no_data_obstacle", no_data_obstacle_, true);
    pr_nh_.param<int>("terrian_analysis/no_data_block_skip_num", no_data_block_skip_num_);
    pr_nh_.param<double>("terrian_analysis/vehicle_height", vehicle_height_);
    pr_nh_.param<int>("terrian_analysis/voxel_point_update_thre", voxel_point_update_thre_);
    pr_nh_.param<double>("terrian_analysis/voxel_time_update_thre", voxel_time_update_thre_);
    pr_nh_.param<double>("terrian_analysis/min_rel_z", min_rel_z_);
    pr_nh_.param<double>("terrian_analysis/max_rel_z", max_rel_z_);
    pr_nh_.param<double>("terrian_analysis/dis_ratio_z", dis_ratio_z_);

    laser_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    laser_cloud_crop_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    laser_cloud_dwz_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    terrain_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    terrain_cloud_elev_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    for(auto &i : terrain_voxel_cloud_){
        i = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    }

    sub_odometry_ = nh_.subscribe<nav_msgs::Odometry>(odometry_topic_.c_str(), 5, &TerrainAnalysis::odometryCallback, this);

    sub_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(cloud_topic_.c_str(), 5, &TerrainAnalysis::pointCloudCallback, this);

    pub_terrian_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/terrian_map", 2);

    down_size_filter_.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);
}

TerrainAnalysis::~TerrainAnalysis()
{

}

void TerrainAnalysis::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    odom_ = *odom;
    double roll, pitch, yaw;
    geometry_msgs::Quaternion q = odom->pose.pose.orientation;
    tf::Matrix3x3 (tf::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);

    // ROS_INFO_STREAM("roll is" << roll << " pitch is" << pitch << " yaw is" << yaw);


    sin_vehicle_roll_ = sin(roll);
    cos_vehicle_roll_ = cos(roll);
    sin_vehicle_pitch_ = sin(pitch);
    cos_vehicle_pitch_ = cos(pitch);
    sin_vehicle_yaw_ = sin(yaw);
    cos_vehicle_yaw_ = cos(yaw);

    if(no_data_inited_ == 0){                          // 第一次接受数据
        vehicle_x_rec_ = odom->pose.pose.position.x;   // 车辆初始位置
        vehicle_y_rec_ = odom->pose.pose.position.y;
        no_data_inited_ = 1;
    }

    if(no_data_inited_ == 1){
        float dis = std::hypot(odom->pose.pose.position.x - vehicle_x_rec_, 
                               odom->pose.pose.position.y - vehicle_y_rec_);
        if(dis > no_decay_dis_)     // 车辆移动距离超过一次里程计雷达监测的距离， 造成点云丢失
            no_data_inited_ == 2;
    }
} 
void TerrainAnalysis::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    if(cloud->width < 100){
        return;
    }

    laser_cloud_time_ = cloud->header.stamp.toSec();  // 将接受到的时间戳转化为秒
    if(!system_inited_){
        system_init_time_ = laser_cloud_time_;
        system_inited_ = true;
    }

    laser_cloud_->clear();
    pcl::fromROSMsg(*cloud, *laser_cloud_);

    pcl::PointXYZI point;
    laser_cloud_crop_->clear();
    int cloud_size = laser_cloud_->points.size();
    for(int i = 0; i < cloud_size; ++i){
        point = laser_cloud_->points[i];
        float point_x = point.x;
        float point_y = point.y;
        float point_z = point.z;

        // 点到车的水平距离
        float dis = std::hypot(point_x - odom_.pose.pose.position.x, point_y - odom_.pose.pose.position.y);
        ROS_INFO_STREAM("dis = " << dis << " dis_z = " << point_z - odom_.pose.pose.position.z);

        if((point_z - odom_.pose.pose.position.z) > min_rel_z_ - dis_ratio_z_ * dis &&
           (point_z - odom_.pose.pose.position.z) < max_rel_z_ + dis_ratio_z_ * dis &&
           dis < terrain_voxel_size_ * (terrain_voxel_half_width_ + 1)){
            point.x = point_x;
            point.y = point_y;
            point.z = point_z;
            point.intensity = laser_cloud_time_ - system_init_time_;
            laser_cloud_crop_->push_back(point);
        }
    }
    ROS_INFO_STREAM("The croped cloud points size is " << laser_cloud_crop_->points.size());

    processPointCloud();
} 

void TerrainAnalysis::processPointCloud()
{
  // terrain voxel roll over
  float terrain_voxel_cen_x = terrain_voxel_size_ * terrain_voxel_shift_x_;
  float terrain_voxel_cen_y = terrain_voxel_size_ * terrain_voxel_shift_y_;

  while (odom_.pose.pose.position.x - terrain_voxel_cen_x < -terrain_voxel_size_) {
    for (int ind_y = 0; ind_y < terrain_voxel_width_; ind_y++) {
      auto terrain_voxel_cloud_ptr =
        terrain_voxel_cloud_[terrain_voxel_width_ * (terrain_voxel_width_ - 1) + ind_y];
      for (int ind_x = terrain_voxel_width_ - 1; ind_x >= 1; ind_x--) {
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y] =
          terrain_voxel_cloud_[terrain_voxel_width_ * (ind_x - 1) + ind_y];
      }
      terrain_voxel_cloud_[ind_y] = terrain_voxel_cloud_ptr;
      terrain_voxel_cloud_[ind_y]->clear();
    }
    terrain_voxel_shift_x_--;
    terrain_voxel_cen_x = terrain_voxel_size_ * terrain_voxel_shift_x_;
  }

  while (odom_.pose.pose.position.x - terrain_voxel_cen_x > terrain_voxel_size_) {
    for (int ind_y = 0; ind_y < terrain_voxel_width_; ind_y++) {
      auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[ind_y];
      for (int ind_x = 0; ind_x < terrain_voxel_width_ - 1; ind_x++) {
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y] =
          terrain_voxel_cloud_[terrain_voxel_width_ * (ind_x + 1) + ind_y];
      }
      terrain_voxel_cloud_[terrain_voxel_width_ * (terrain_voxel_width_ - 1) + ind_y] =
        terrain_voxel_cloud_ptr;
      terrain_voxel_cloud_[terrain_voxel_width_ * (terrain_voxel_width_ - 1) + ind_y]->clear();
    }
    terrain_voxel_shift_x_++;
    terrain_voxel_cen_x = terrain_voxel_size_ * terrain_voxel_shift_x_;
  }

  while (odom_.pose.pose.position.y - terrain_voxel_cen_y < -terrain_voxel_size_) {
    for (int ind_x = 0; ind_x < terrain_voxel_width_; ind_x++) {
      auto terrain_voxel_cloud_ptr =
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (terrain_voxel_width_ - 1)];
      for (int ind_y = terrain_voxel_width_ - 1; ind_y >= 1; ind_y--) {
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y] =
          terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (ind_y - 1)];
      }
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x] = terrain_voxel_cloud_ptr;
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x]->clear();
    }
    terrain_voxel_shift_y_--;
    terrain_voxel_cen_y = terrain_voxel_size_ * terrain_voxel_shift_y_;
  }

  while (odom_.pose.pose.position.y - terrain_voxel_cen_y > terrain_voxel_size_) {
    for (int ind_x = 0; ind_x < terrain_voxel_width_; ind_x++) {
      auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[terrain_voxel_width_ * ind_x];
      for (int ind_y = 0; ind_y < terrain_voxel_width_ - 1; ind_y++) {
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y] =
          terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (ind_y + 1)];
      }
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (terrain_voxel_width_ - 1)] =
        terrain_voxel_cloud_ptr;
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (terrain_voxel_width_ - 1)]->clear();
    }
    terrain_voxel_shift_y_++;
    terrain_voxel_cen_y = terrain_voxel_size_ * terrain_voxel_shift_y_;
  }

  // stack registered laser scans
  pcl::PointXYZI point;
  int laser_cloud_crop_size = laser_cloud_crop_->points.size();
  for (int i = 0; i < laser_cloud_crop_size; i++) {
    point = laser_cloud_crop_->points[i];

    int ind_x =
      static_cast<int>(
        (point.x - odom_.pose.pose.position.x + terrain_voxel_size_ / 2) / terrain_voxel_size_) +
      terrain_voxel_half_width_;
    int ind_y =
      static_cast<int>(
        (point.y - odom_.pose.pose.position.y + terrain_voxel_size_ / 2) / terrain_voxel_size_) +
      terrain_voxel_half_width_;

    if (point.x - odom_.pose.pose.position.x + terrain_voxel_size_ / 2 < 0) ind_x--;
    if (point.y - odom_.pose.pose.position.y + terrain_voxel_size_ / 2 < 0) ind_y--;

    if (ind_x >= 0 && ind_x < terrain_voxel_width_ && ind_y >= 0 && ind_y < terrain_voxel_width_) {
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y]->push_back(point);
      terrain_voxel_update_num_[terrain_voxel_width_ * ind_x + ind_y]++;
    }
  }

  for (int ind = 0; ind < terrain_voxel_num_; ind++) {
    if (
      terrain_voxel_update_num_[ind] >= voxel_point_update_thre_ ||
      laser_cloud_time_ - system_init_time_ - terrain_voxel_update_time_[ind] >=
        voxel_time_update_thre_ ||
      clearing_cloud_) {
      auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[ind];

      laser_cloud_dwz_->clear();
      down_size_filter_.setInputCloud(terrain_voxel_cloud_ptr);
      down_size_filter_.filter(*laser_cloud_dwz_);

      terrain_voxel_cloud_ptr->clear();
      int laser_cloud_dwz_size = laser_cloud_dwz_->points.size();
      for (int i = 0; i < laser_cloud_dwz_size; i++) {
        point = laser_cloud_dwz_->points[i];
        float dis = sqrt(
          (point.x - odom_.pose.pose.position.x) * (point.x - odom_.pose.pose.position.x) +
          (point.y - odom_.pose.pose.position.y) * (point.y - odom_.pose.pose.position.y));
        if (
          point.z - odom_.pose.pose.position.z > min_rel_z_ - dis_ratio_z_ * dis &&
          point.z - odom_.pose.pose.position.z < max_rel_z_ + dis_ratio_z_ * dis &&
          (laser_cloud_time_ - system_init_time_ - point.intensity < decay_time_ ||
           dis < no_decay_dis_) &&
          (dis >= clearing_dis_ || !clearing_cloud_)) {
          terrain_voxel_cloud_ptr->push_back(point);
        }
      }

      terrain_voxel_update_num_[ind] = 0;
      terrain_voxel_update_time_[ind] = laser_cloud_time_ - system_init_time_;
    }
  }

  terrain_cloud_->clear();
  for (int ind_x = terrain_voxel_half_width_ - 5; ind_x <= terrain_voxel_half_width_ + 5; ind_x++) {
    for (int ind_y = terrain_voxel_half_width_ - 5; ind_y <= terrain_voxel_half_width_ + 5;
         ind_y++) {
      *terrain_cloud_ += *terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y];
    }
  }

  // estimate ground and compute elevation for each point
  for (int i = 0; i < planar_voxel_num_; i++) {
    planar_voxel_elev_[i] = 0;
    planar_voxel_edge_[i] = 0;
    planar_voxel_dy_obs_[i] = 0;
    planar_point_elev_[i].clear();
  }

  int terrain_cloud_size = terrain_cloud_->points.size();
  for (int i = 0; i < terrain_cloud_size; i++) {
    point = terrain_cloud_->points[i];

    int ind_x =
      static_cast<int>(
        (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2) / planar_voxel_size_) +
      planar_voxel_half_width_;
    int ind_y =
      static_cast<int>(
        (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2) / planar_voxel_size_) +
      planar_voxel_half_width_;

    if (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2 < 0) ind_x--;
    if (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2 < 0) ind_y--;

    if (
      point.z - odom_.pose.pose.position.z > min_rel_z_ &&
      point.z - odom_.pose.pose.position.z < max_rel_z_) {
      for (int d_x = -1; d_x <= 1; d_x++) {
        for (int d_y = -1; d_y <= 1; d_y++) {
          if (
            ind_x + d_x >= 0 && ind_x + d_x < planar_voxel_width_ && ind_y + d_y >= 0 &&
            ind_y + d_y < planar_voxel_width_) {
            planar_point_elev_[planar_voxel_width_ * (ind_x + d_x) + ind_y + d_y].push_back(
              point.z);
          }
        }
      }
    }

    if (clear_dy_obs_) {
      if (ind_x >= 0 && ind_x < planar_voxel_width_ && ind_y >= 0 && ind_y < planar_voxel_width_) {
        float point_x1 = point.x - odom_.pose.pose.position.x;
        float point_y1 = point.y - odom_.pose.pose.position.y;
        float point_z1 = point.z - odom_.pose.pose.position.z;

        float dis1 = sqrt(point_x1 * point_x1 + point_y1 * point_y1);
        if (dis1 > min_dy_obs_dis_) {
          float angle1 = atan2(point_z1 - min_dy_obs_rel_z_, dis1) * 180.0 / M_PI;
          if (angle1 > min_dy_obs_angle_) {
            float point_x2 = point_x1 * cos_vehicle_yaw_ + point_y1 * sin_vehicle_yaw_;
            float point_y2 = -point_x1 * sin_vehicle_yaw_ + point_y1 * cos_vehicle_yaw_;
            float point_z2 = point_z1;

            float point_x3 = point_x2 * cos_vehicle_pitch_ - point_z2 * sin_vehicle_pitch_;
            float point_y3 = point_y2;
            float point_z3 = point_x2 * sin_vehicle_pitch_ + point_z2 * cos_vehicle_pitch_;

            float point_x4 = point_x3;
            float point_y4 = point_y3 * cos_vehicle_roll_ + point_z3 * sin_vehicle_roll_;
            float point_z4 = -point_y3 * sin_vehicle_roll_ + point_z3 * cos_vehicle_roll_;

            float dis4 = sqrt(point_x4 * point_x4 + point_y4 * point_y4);
            float angle4 = atan2(point_z4, dis4) * 180.0 / M_PI;
            if (
              (angle4 > min_dy_obs_vfov_ && angle4 < max_dy_obs_vfov_) ||
              fabs(point_z4) < abs_dy_obs_rel_z_thre_) {
              planar_voxel_dy_obs_[planar_voxel_width_ * ind_x + ind_y]++;
            }
          }
        } else {
          planar_voxel_dy_obs_[planar_voxel_width_ * ind_x + ind_y] += min_dy_obs_point_num_;
        }
      }
    }
  }

  if (clear_dy_obs_) {
    for (int i = 0; i < laser_cloud_crop_size; i++) {
      point = laser_cloud_crop_->points[i];

      int ind_x =
        static_cast<int>(
          (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2) / planar_voxel_size_) +
        planar_voxel_half_width_;
      int ind_y =
        static_cast<int>(
          (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2) / planar_voxel_size_) +
        planar_voxel_half_width_;

      if (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2 < 0) ind_x--;
      if (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2 < 0) ind_y--;

      if (ind_x >= 0 && ind_x < planar_voxel_width_ && ind_y >= 0 && ind_y < planar_voxel_width_) {
        float point_x1 = point.x - odom_.pose.pose.position.x;
        float point_y1 = point.y - odom_.pose.pose.position.y;
        float point_z1 = point.z - odom_.pose.pose.position.z;

        float dis1 = sqrt(point_x1 * point_x1 + point_y1 * point_y1);
        float angle1 = atan2(point_z1 - min_dy_obs_rel_z_, dis1) * 180.0 / M_PI;
        if (angle1 > min_dy_obs_angle_) {
          planar_voxel_dy_obs_[planar_voxel_width_ * ind_x + ind_y] = 0;
        }
      }
    }
  }

  if (use_sorting_) {
    for (int i = 0; i < planar_voxel_num_; i++) {
      int planar_point_elev_size = planar_point_elev_[i].size();
      if (planar_point_elev_size > 0) {
        std::sort(planar_point_elev_[i].begin(), planar_point_elev_[i].end());

        int quantile_id = static_cast<int>(quantile_z_ * planar_point_elev_size);
        if (quantile_id < 0)
          quantile_id = 0;
        else if (quantile_id >= planar_point_elev_size)
          quantile_id = planar_point_elev_size - 1;

        if (
          planar_point_elev_[i][quantile_id] > planar_point_elev_[i][0] + max_ground_lift_ &&
          limit_ground_lift_) {
          planar_voxel_elev_[i] = planar_point_elev_[i][0] + max_ground_lift_;
        } else {
          planar_voxel_elev_[i] = planar_point_elev_[i][quantile_id];
        }
      }
    }
  } else {
    for (int i = 0; i < planar_voxel_num_; i++) {
      int planar_point_elev_size = planar_point_elev_[i].size();
      if (planar_point_elev_size > 0) {
        float min_z = 1000.0;
        int min_id = -1;
        for (int j = 0; j < planar_point_elev_size; j++) {
          if (planar_point_elev_[i][j] < min_z) {
            min_z = planar_point_elev_[i][j];
            min_id = j;
          }
        }

        if (min_id != -1) {
          planar_voxel_elev_[i] = planar_point_elev_[i][min_id];
        }
      }
    }
  }

  terrain_cloud_elev_->clear();
  int terrain_cloud_elev_size = 0;
  for (int i = 0; i < terrain_cloud_size; i++) {
    point = terrain_cloud_->points[i];
    if (
      point.z - odom_.pose.pose.position.z > min_rel_z_ &&
      point.z - odom_.pose.pose.position.z < max_rel_z_) {
      int ind_x =
        static_cast<int>(
          (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2) / planar_voxel_size_) +
        planar_voxel_half_width_;
      int ind_y =
        static_cast<int>(
          (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2) / planar_voxel_size_) +
        planar_voxel_half_width_;

      if (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2 < 0) ind_x--;
      if (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2 < 0) ind_y--;

      if (ind_x >= 0 && ind_x < planar_voxel_width_ && ind_y >= 0 && ind_y < planar_voxel_width_) {
        if (
          planar_voxel_dy_obs_[planar_voxel_width_ * ind_x + ind_y] < min_dy_obs_point_num_ ||
          !clear_dy_obs_) {
          float dis_z = point.z - planar_voxel_elev_[planar_voxel_width_ * ind_x + ind_y];
          if (consider_drop_) dis_z = fabs(dis_z);
          int planar_point_elev_size =
            planar_point_elev_[planar_voxel_width_ * ind_x + ind_y].size();
          if (
            dis_z >= 0.1 && dis_z < vehicle_height_ &&
            planar_point_elev_size >= min_block_point_num_) {
            terrain_cloud_elev_->push_back(point);
            terrain_cloud_elev_->points[terrain_cloud_elev_size].intensity = dis_z;

            terrain_cloud_elev_size++;
          }
        }
      }
    }
  }

  if (no_data_obstacle_ && no_data_inited_ == 2) {
    for (int i = 0; i < planar_voxel_num_; i++) {
      int planar_point_elev_size = planar_point_elev_[i].size();
      if (planar_point_elev_size < min_block_point_num_) {
        planar_voxel_edge_[i] = 1;
      }
    }

    for (int no_data_block_skip_count = 0; no_data_block_skip_count < no_data_block_skip_num_;
         no_data_block_skip_count++) {
      for (int i = 0; i < planar_voxel_num_; i++) {
        if (planar_voxel_edge_[i] >= 1) {
          int ind_x = static_cast<int>(i / planar_voxel_width_);
          int ind_y = i % planar_voxel_width_;
          bool edge_voxel = false;
          for (int d_x = -1; d_x <= 1; d_x++) {
            for (int d_y = -1; d_y <= 1; d_y++) {
              if (
                ind_x + d_x >= 0 && ind_x + d_x < planar_voxel_width_ && ind_y + d_y >= 0 &&
                ind_y + d_y < planar_voxel_width_) {
                if (
                  planar_voxel_edge_[planar_voxel_width_ * (ind_x + d_x) + ind_y + d_y] <
                  planar_voxel_edge_[i]) {
                  edge_voxel = true;
                }
              }
            }
          }

          if (!edge_voxel) planar_voxel_edge_[i]++;
        }
      }
    }

    for (int i = 0; i < planar_voxel_num_; i++) {
      if (planar_voxel_edge_[i] > no_data_block_skip_num_) {
        int ind_x = static_cast<int>(i / planar_voxel_width_);
        int ind_y = i % planar_voxel_width_;

        point.x =
          planar_voxel_size_ * (ind_x - planar_voxel_half_width_) + odom_.pose.pose.position.x;
        point.y =
          planar_voxel_size_ * (ind_y - planar_voxel_half_width_) + odom_.pose.pose.position.y;
        point.z = odom_.pose.pose.position.z;
        point.intensity = vehicle_height_;

        point.x -= planar_voxel_size_ / 4.0;
        point.y -= planar_voxel_size_ / 4.0;
        terrain_cloud_elev_->push_back(point);

        point.x += planar_voxel_size_ / 2.0;
        terrain_cloud_elev_->push_back(point);

        point.y += planar_voxel_size_ / 2.0;
        terrain_cloud_elev_->push_back(point);

        point.x -= planar_voxel_size_ / 2.0;
        terrain_cloud_elev_->push_back(point);
      }
    }
  }

  clearing_cloud_ = false;

  if(terrain_cloud_elev_->points.size() < 1){
    ROS_WARN("The terrain cloud has no enough points!");
  } else{
    ROS_INFO_STREAM("The terrain cloud points size is " << terrain_cloud_elev_->points.size());
  }
  // Publish points with elevation
  sensor_msgs::PointCloud2 terrain_cloud;
  pcl::toROSMsg(*terrain_cloud_elev_, terrain_cloud);
  terrain_cloud.header.stamp = ros::Time().fromSec(laser_cloud_time_);
  terrain_cloud.header.frame_id = sensor_frame_;

  // Transform point cloud to lidar frame
//   sensor_msgs::PointCloud2 terrain_cloud_lidar;
//   tf::Transform tf_odom_to_lidar;
//   tf_odom_to_lidar.setOrigin(std::vector(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z));
//   (odom_.pose.pose, tf_odom_to_lidar);
//   pcl_ros::transformPointCloud(
//     sensor_frame_, tf_odom_to_lidar.inverse(), terrain_cloud, terrain_cloud_lidar);

  // Publish the transformed point cloud
  pub_terrian_map_.publish(terrain_cloud);
}
int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "terrain_analysis");
    TerrainAnalysis terrain_analysis;

    // run with two threads
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}

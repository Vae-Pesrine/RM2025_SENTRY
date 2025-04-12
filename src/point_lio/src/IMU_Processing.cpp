#include "IMU_Processing.h"

//根据点的曲率对点进行排序
const bool time_list(PointType &x, PointType &y) 
{
  return (x.curvature < y.curvature);
};

//设置陀螺仪的协防差缩放因子
void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

//设置加速度计的协方差缩放因子
void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_vel_scale = scaler;
}

//初始化IMU处理对象的成员变量
ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true)       //是否是第一帧   是否需要初始化
{
  imu_en = true;                       //IMU是否启用
  init_iter_num = 1;                   //初始化迭代次数
  mean_acc      = V3D(0, 0, 0.0);      //存储加速度计和陀螺仪的平均值
  mean_gyr      = V3D(0, 0, 0);       
  after_imu_init_ = false;             //IMU是否初始化完成
  state_cov.setIdentity();             //状态协方差矩阵， 初始化为单位矩阵
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, 0.0);
  mean_gyr      = V3D(0, 0, 0);
  imu_need_init_    = true;
  init_iter_num     = 1;
  after_imu_init_   = false;
  
  time_last_scan = 0.0;
}


// 设置初始重力和旋转矩阵。
// 计算重力的方向，并根据重力方向计算旋转矩阵。
void ImuProcess::Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  // V3D tmp_gravity = - mean_acc / mean_acc.norm() * G_m_s2; // state_gravity;
  M3D hat_grav;
  hat_grav << 0.0, gravity_(2), -gravity_(1),
              -gravity_(2), 0.0, gravity_(0),
              gravity_(1), -gravity_(0), 0.0;
  double align_norm = (hat_grav * tmp_gravity).norm() / gravity_.norm() / tmp_gravity.norm();
  double align_cos = gravity_.transpose() * tmp_gravity;
  align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();
  if (align_norm < 1e-6)
  {
    if (align_cos > 1e-6)
    {
      rot = Eye3d;
    }
    else
    {
      rot = -Eye3d;
    }
  }
  else
  {
    V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos); 
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
  }
}

// 初始化IMU。
// 计算加速度计和陀螺仪的平均值。
// 如果是第一帧，重置IMU处理对象的状态，并初始化加速度计和陀螺仪的平均值。
// 遍历IMU测量数据，更新加速度计和陀螺仪的平均值。
void ImuProcess::IMU_init(const MeasureGroup &meas, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    N ++;
  }
}

// 处理IMU和激光雷达数据。
// 如果IMU启用且IMU测量数据不为空，进行IMU初始化。
// 如果IMU需要初始化，调用IMU_init函数进行初始化。
// 如果初始化完成，将当前激光雷达点云数据赋值给cur_pcl_un_。
// 如果IMU未启用，直接将激光雷达点云数据赋值给cur_pcl_un_
void ImuProcess::Process(const MeasureGroup &meas, PointCloudXYZI::Ptr cur_pcl_un_)
{  
  if (imu_en)
  {
    if(meas.imu.empty())  return;

    if (imu_need_init_)
    {
      
      {
        /// The very first lidar frame
        IMU_init(meas, init_iter_num);

        imu_need_init_ = true;

        if (init_iter_num > MAX_INI_COUNT)
        {
          ROS_INFO("IMU Initializing: %.1f %%", 100.0);
          imu_need_init_ = false;
          *cur_pcl_un_ = *(meas.lidar);
        }
        // *cur_pcl_un_ = *(meas.lidar);
      }
      return;
    }
    if (!after_imu_init_) after_imu_init_ = true;
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
  else
  {
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
}
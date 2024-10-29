#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <memory>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>

namespace sentry_localization
{

template<typename T, class System> class UnscentedKalmanFilterX;

class PoseSystem;
class OdomSystem;                                                                                                                                                            

class PoseEstimator 
{
public:
    //点云配准方法 初始位置 初始方向
    PoseEstimator(pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr& registration, const Eigen::Vector3f& pose, const Eigen::Quaternionf& quat, double time_duration = 1.0);
    ~PoseEstimator();

    void predict(const ros::Time& stamp);

    //使用加速度计和角速度数据更新位置估计
    void predict(const ros::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro);

    //使用里程计数据更新里程计姿态估计
    void predict_odom(const Eigen::Matrix4f& odom_delta);

    //使用点云数据校正位置估计， 返回对齐的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr correct(const ros::Time& stamp, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);

    ros::Time last_correction_time() const;

    Eigen::Vector3f pose() const;
    Eigen::Vector3f vel() const;
    Eigen::Quaternionf quat() const;
    Eigen::Matrix4f matrix() const;                  
    Eigen::Vector3f odom_pose() const;
    Eigen::Quaternionf odom_quat() const;
    Eigen::Matrix4f odom_matrix() const;

    const boost::optional<Eigen::Matrix4f>& wo_prediction_error() const;
    const boost::optional<Eigen::Matrix4f>& imu_prediction_error() const;
    const boost::optional<Eigen::Matrix4f>& odom_prediction_error() const;

private:
    ros::Time initial_stamp;              //初始化时间戳
    ros::Time prev_stamp;                  //上次更新的时间戳
    ros::Time last_correction_stamp;       //上次校正的时间戳
    double time_duration;                  //在该时间段不进行预测

    Eigen::Matrix4f process_noise;         //过程噪声
    std::unique_ptr<sentry_localization::UnscentedKalmanFilterX<float, PoseSystem>> ukf;        //基于无迹卡尔曼滤波器的姿态估计器
    std::unique_ptr<sentry_localization::UnscentedKalmanFilterX<float, OdomSystem>> odom_ukf;   //基于无迹卡尔曼滤波器的里程计姿态估计器

    Eigen::Matrix4f last_observation;                                       //上次观测值
    boost::optional<Eigen::Matrix4f> wo_pred_error;                   //不使用IMU的位姿估计误差
    boost::optional<Eigen::Matrix4f> imu_pred_error;                  //使用IMU的位姿估计
    boost::optional<Eigen::Matrix4f> odom_pred_error;                 //里程计姿态估计误差

    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;    //点云配准方法
};

}
#endif
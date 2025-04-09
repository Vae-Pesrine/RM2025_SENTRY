// common
#include <ros/ros.h>
#include <iostream>
#include <queue>

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>              // 直通滤波
#include <pcl/filters/extract_indices.h>          // 按索引提取点云
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>      // 条件滤波
#include <pcl/segmentation/sac_segmentation.h>    // RANSAC分割算法
#include <pcl/segmentation/extract_clusters.h>    // 欧几里得聚类

// msgs
#include <sensor_msgs/PointCloud2.h>


// typedef 
using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

class CloudSegmenter {
public:
    CloudSegmenter();
    ~CloudSegmenter();

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg);
    void processCombinedCloud(const PointCloud::Ptr& input_msg); 


private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_cloud_;
    ros::Publisher pub_ground_, pub_obstacle_;
    
    double max_ground_angle_;    // 最大地面角度(度)
    double segmentation_point_distance;
    double cluster_tolerance_;   // 聚类容差(m)
    int min_cluster_size_;       // 最小聚类点数
    double limit_distance_;
    double ground_height_;

    std::queue<PointCloud::Ptr> cloud_queue_;
    const size_t frame_count_ = 10;  // 要累积的帧数

    pcl::SACSegmentation<PointT> seg;                           
    pcl::ConditionalRemoval<PointT> cnondition_removal;   
    pcl::PassThrough<PointT> pass_through;

    std::string map_frame_ = "map";


};


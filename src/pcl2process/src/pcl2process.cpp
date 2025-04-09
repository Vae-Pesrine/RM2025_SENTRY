#include "pcl2process.hpp"

/**
 * @brief the init func, pass params and set the filters
 */
CloudSegmenter::CloudSegmenter() : nh_("~") 
{
    nh_.param<double>("max_ground_angle", max_ground_angle_, 10.0); // 最大地面倾角(度)
    nh_.param<double>("segmentation_point_distance", segmentation_point_distance, 0.1);
    nh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.5); // 聚类容差(m)
    nh_.param<int>("min_cluster_size", min_cluster_size_, 100); // 最小聚类点数
    nh_.param<double>("limit_distance", limit_distance_, 10.0);
    nh_.param<double>("ground_height", ground_height_, 0.2);

    ROS_INFO_STREAM("The max_ground_angle is " << max_ground_angle_);

    // subscribe and publish
    sub_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/livox/points", 1, 
        &CloudSegmenter::cloudCallback, this);
    pub_ground_ = nh_.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
    pub_obstacle_ = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_cloud", 1);
    
    // RANSACsegmentation
    seg.setOptimizeCoefficients(true);                     // 使用系数优化
    // seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);   // 设置平面模型
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);                    // 使用RANSAC
    seg.setMaxIterations(1000);                            // 最大迭代次数
    seg.setDistanceThreshold(segmentation_point_distance); // 内点距离阈值
    seg.setAxis(Eigen::Vector3f::UnitZ());                 // 设置法向量约束
    seg.setEpsAngle(max_ground_angle_ * M_PI / 180.0);     // 设置最大地面角度
    
    // Condition Removal
    auto outside = std::make_shared<pcl::ConditionAnd<PointT>>();
    outside->addComparison(std::make_shared<const pcl::FieldComparison<PointT>>(
        "x", pcl::ComparisonOps::GT, -limit_distance_ / 2.0));
    outside->addComparison(std::make_shared<const pcl::FieldComparison<PointT>>(
        "x", pcl::ComparisonOps::LT, limit_distance_ / 2.0));
    outside->addComparison(std::make_shared<const pcl::FieldComparison<PointT>>(
        "y", pcl::ComparisonOps::GT, -limit_distance_ / 2.0));
    outside->addComparison(std::make_shared<const pcl::FieldComparison<PointT>>(
        "y", pcl::ComparisonOps::LT, limit_distance_ / 2.0));
    cnondition_removal.setCondition(outside);

    // pass through
    pass_through.setFilterFieldName("z");
    pass_through.setFilterLimits(0, ground_height_);
}

CloudSegmenter::~CloudSegmenter()
{

}

void CloudSegmenter::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg) 
{
    auto input_cloud = std::make_shared<PointCloud>();
    pcl::fromROSMsg(*input_msg, *input_cloud);

    // cnondition_removal.setInputCloud(input_cloud);
    // cnondition_removal.filter(*input_cloud);

    // auto indices_removed_xyz = std::make_shared<pcl::PointIndices>();
    // pass_through.setInputCloud(input_cloud);
    // pass_through.filter(*input_cloud);
    // pass_through.filter(indices_removed_xyz->indices);

    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto indices_plane = std::make_shared<pcl::PointIndices>();
    seg.setInputCloud(input_cloud);
    seg.segment(*indices_plane, *coefficients);                  // 输出内点和平面系数
    
    // auto indices_original = std::make_shared<pcl::PointIndices>();
    // for(auto index : indices_plane->indices){
    //     if(index >= 0 && index < indices_removed_xyz->indices.size()){
    //         indices_original->indices.push_back(indices_removed_xyz->indices[index]);
    //     }
    // }

    auto extract = pcl::ExtractIndices<PointT>{};
    auto ground_cloud = std::make_shared<PointCloud>();
    auto non_ground_cloud = std::make_shared<PointCloud>();

    extract.setInputCloud(input_cloud);
    extract.setIndices(indices_plane);
    extract.setNegative(false);
    extract.filter(*ground_cloud);
    
    // // 2. 障碍物聚类 (欧几里得聚类)
    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    // tree->setInputCloud(non_ground_cloud);
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance(cluster_tolerance_);
    // ec.setMinClusterSize(min_cluster_size_);
    // ec.setMaxClusterSize(25000);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(non_ground_cloud);
    // ec.extract(cluster_indices);
    // // 合并所有障碍物点云
    // PointCloud::Ptr obstacle_cloud(new PointCloud);
    // for (const auto& indices : cluster_indices) {
    //     for (const auto& idx : indices.indices) {
    //         obstacle_cloud->points.push_back(non_ground_cloud->points[idx]);
    //     }
    // }

    // publish the result
    sensor_msgs::PointCloud2 ground_msg, obstacle_msg;
    pcl::toROSMsg(*ground_cloud, ground_msg);
    
    ground_msg.header = input_msg->header;
    
    pub_ground_.publish(ground_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_segmenter");
    CloudSegmenter segmenter;
    ros::spin();
    return 0;
}
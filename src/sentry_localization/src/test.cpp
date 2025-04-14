#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>


using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using SurfaceNormals = pcl::PointCloud<pcl::Normal>;
using LocalFeatures = pcl::PointCloud<pcl::FPFHSignature33>;
using SearchKdTree = pcl::search::KdTree<pcl::PointXYZ>;

class FeatureCloud
{
private:
    // 保存点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_;
    // 表面法向量信息
    SurfaceNormals::Ptr normals_;
    // 特征描述子信息 快速点特征直方图
    LocalFeatures::Ptr features_;
    // 邻域搜索方法 KDTree
    SearchKdTree::Ptr search_method_xyz_;
    /**
     * @brief 搜索半径，normal是计算法线的半径，feature是计算特征描述子的半径，
     * 后者大一些
     */
    float normal_radius_;
    float feature_radius_;

public:
    FeatureCloud()
        : search_method_xyz_(new SearchKdTree), 
          normal_radius_(1.2f), 
          feature_radius_(1.21f){
    }

    ~FeatureCloud(){
    }

    PointCloud::Ptr getPointCloud() { return xyz_; }

    SurfaceNormals::Ptr getSurfaceNormals() { return normals_; }

    LocalFeatures::Ptr getLocalFeatures()   { return features_; }

    /**
     * @brief compute the normals
     *        input cloud: xyz_    output normals: normals_
     */
    void computeSurfaceNormals(){
        normals_ = SurfaceNormals::Ptr(new SurfaceNormals);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal;
        normal.setInputCloud(xyz_);
        normal.setSearchMethod(search_method_xyz_);
        normal.setRadiusSearch(normal_radius_);
        normal.compute(*normals_);
    }

    void computeLocalFeatures(){

        features_ = LocalFeatures::Ptr(new LocalFeatures);
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud(xyz_);
        fpfh.setInputNormals(normals_);
        fpfh.setSearchMethod(search_method_xyz_);
        fpfh.setRadiusSearch(feature_radius_);
        fpfh.compute(*features_);
    }

    /**
     * @ load pointcloud by loading pcd file
     */
    void loadInputCloud(const std::string &pcd_filename){
        xyz_ = PointCloud::Ptr(new PointCloud);

        if(pcl::io::loadPCDFile(pcd_filename, *xyz_) == -1){
            PCL_ERROR("Couldn't read pcd file");
            return;
        }

        computeSurfaceNormals();
        computeLocalFeatures();
    }

    /**
     * @brief load pointcloud directly
     */
    void setInputCloud(PointCloud::Ptr cloud){
        xyz_ = cloud;
        computeSurfaceNormals();
        computeLocalFeatures();        
    }

};

class TemplateAlignment
{
private:
    std::vector<FeatureCloud> source_;
    FeatureCloud target_;

    pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia_;

    float transform_epsilon_;

    float min_sample_distance_;

    float max_correspondence_distance_;

    float max_iterations_;

public:
    struct Result{
        float fitness_score;
        int hasConverged;
        Eigen::Matrix4f transformation;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };  

public:
    TemplateAlignment()
        : transform_epsilon_(0.001f),
          min_sample_distance_(0.01f),
          max_correspondence_distance_(10.0f),
          max_iterations_(5000){
        sac_ia_.setTransformationEpsilon(transform_epsilon_);
        sac_ia_.setMinSampleDistance(min_sample_distance_);
        sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
        sac_ia_.setMaximumIterations(max_iterations_);
    }

    ~TemplateAlignment(){
    }

    void addSourceCloud(FeatureCloud &cloud){
        source_.push_back(cloud);
    }

    void setTargetCloud(FeatureCloud &cloud){
        target_ = cloud;
        sac_ia_.setInputTarget(cloud.getPointCloud());
        sac_ia_.setTargetFeatures(cloud.getLocalFeatures());
    }

    void align(FeatureCloud &source_cloud, TemplateAlignment::Result &result){
        PointCloud registration_cloud;
        sac_ia_.setInputTarget(source_cloud.getPointCloud());
        sac_ia_.setSourceFeatures(source_cloud.getLocalFeatures());
        sac_ia_.align(registration_cloud);

        result.fitness_score = static_cast<float>(sac_ia_.getFitnessScore(max_correspondence_distance_));
        result.transformation = sac_ia_.getFinalTransformation();
        result.hasConverged = sac_ia_.hasConverged();
    }

    void align(FeatureCloud &source_cloud, TemplateAlignment::Result &result,pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 &T_prev){
        PointCloud registration_output;
        sac_ia_.setInputSource(source_cloud.getPointCloud());
        sac_ia_.setSourceFeatures(source_cloud.getLocalFeatures());
        sac_ia_.align(registration_output,T_prev);

        result.fitness_score = (float) sac_ia_.getFitnessScore(max_correspondence_distance_);
        result.transformation = sac_ia_.getFinalTransformation();
        result.hasConverged = sac_ia_.hasConverged();
    };    

    void alignAll(std::vector<Result, Eigen::aligned_allocator<Result>>& results,
                  pcl::registration::TransformationEstimation<PointT, PointT>::Matrix4 &T_prev){//Eigen下提供了，数据分配符，数据对齐aligned_allocator
        results.resize(source_.size());
        for (int i = 0; i < source_.size(); ++i) {
            align(source_[i], results[i],T_prev);
        }
    };

    int findBestAligment(TemplateAlignment::Result &result,pcl::registration::TransformationEstimation<PointT, PointT>::Matrix4 &T_pre){
        std::vector<Result, Eigen::aligned_allocator<Result>> results;
        alignAll(results,T_pre);
        
        auto best_result_iter = std::min_element(results.begin(), results.end(), 
        [](const Result &a, const Result &b) {        
            return a.fitness_score < b.fitness_score;
        });

        int best_template_index = std::distance(results.begin(), best_result_iter);

        result = *best_result_iter;
        return best_template_index;
    };
};


int main(int argc, char *argv[])
{
    std::string pcd_file_name = "/home/jgy/RMUC_simulation/src/sentry_localization/PCD/room_scan1.pcd";

    PointCloud::Ptr target_cloud(new PointCloud);
    PointCloud::Ptr source_cloud(new PointCloud);

    // 加载目标点云
    if (pcl::io::loadPCDFile<PointT>(pcd_file_name, *target_cloud) == -1) {
        std::cerr << "Couldn't read file " << pcd_file_name << std::endl;
        return -1;
    }

    // 检查目标点云是否为空
    if (target_cloud->points.empty()) {
        std::cerr << "Target cloud is empty!" << std::endl;
        return -1;
    }

    // 添加体素滤波
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(target_cloud);
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f); // 设置体素大小
    PointCloud::Ptr filtered_target_cloud(new PointCloud);
    voxel_filter.filter(*filtered_target_cloud);

    // 检查滤波后的点云是否为空
    if (filtered_target_cloud->points.empty()) {
        std::cerr << "Filtered target cloud is empty!" << std::endl;
        return -1;
    }

    // 使用滤波后的点云作为目标点云
    target_cloud = filtered_target_cloud;
    std::cout << "Loaded target cloud with " << target_cloud->points.size() << " points." << std::endl;

    // 创建源点云并添加偏移
    for (auto &point : target_cloud->points) {
        PointT new_point = point;
        new_point.x += 4.0f;
        new_point.y += 6.0f;
        new_point.z += 0.2f;
        source_cloud->points.push_back(new_point);
    }

    // 检查源点云是否为空
    if (source_cloud->points.empty()) {
        std::cerr << "Source cloud is empty!" << std::endl;
        return -1;
    }

    std::cout << "align begins..." << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();
    // 初始化目标点云和源点云
    FeatureCloud target;
    target.setInputCloud(target_cloud);

    FeatureCloud source;
    source.setInputCloud(source_cloud);

    TemplateAlignment align;
    align.setTargetCloud(target);
    align.addSourceCloud(source);

    TemplateAlignment::Result result;

    pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 T_prev = Eigen::Matrix4f::Identity();
    float score;
    try {
        int best_index = align.findBestAligment(result, T_prev);
        score = result.fitness_score;

        auto end_time = std::chrono::high_resolution_clock::now();

        std::cout << "*********** RANSAC 结果 ***********" << std::endl;
        std::cout << "是否收敛: " << result.hasConverged << std::endl;
        std::cout << "匹配分数: " << result.fitness_score << " (索引: " << best_index << ")" << std::endl;
        std::cout << "变换矩阵:\n" << result.transformation << std::endl;
        std::cout << "匹配时间: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " ms" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Alignment failed: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
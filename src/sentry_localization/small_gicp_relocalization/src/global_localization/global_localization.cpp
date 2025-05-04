#include "global_localization.h"
#include "tools.h"
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

namespace global_localization {

    GlobalLocalizationBBS::GlobalLocalizationBBS(ros::NodeHandle& nh): nh_(nh) {
        float ndt_resolution, ndt_step_size, ndt_epsilon, ndt_max_iterations;
        nh_.getParam("ndt/resolution", ndt_resolution);
        nh_.getParam("ndt/step_size", ndt_step_size);
        nh_.getParam("ndt/epsilon", ndt_epsilon);
        nh_.getParam("ndt/max_iterations", ndt_max_iterations);
        ndt.setResolution(ndt_resolution);
        ndt.setStepSize(ndt_step_size);
        ndt.setTransformationEpsilon(ndt_epsilon);
        ndt.setMaximumIterations(ndt_max_iterations);
    }

    GlobalLocalizationBBS ::~GlobalLocalizationBBS() {}

    // raw map
    void GlobalLocalizationBBS::set_global_map(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud){
        map_ = cloud;
    }

    bool GlobalLocalizationBBS::globalLocalization(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, const std::string& predict_txt, Eigen::Matrix4f& result){
        // 1. read path from txt
        Eigen::Matrix4f T;
        bool read_success = read_position(predict_txt, T);
        if (!read_success ||
            T.block<3, 3>(0,0).determinant() < 0.8 ||
            T.block<3, 3>(0,0).determinant() == NAN ||
            T.block<3, 3>(0,0).determinant() > 1.2 ||
            T(0, 3) == NAN || T(1, 3) == NAN || T(2, 3) == NAN ){
            // T is not available
            return globalLocalization(cloud, result);
        }
        else{
            // match by ndt
            pcl::PointCloud<pcl::PointXYZI> align;
            ndt.setInputTarget(map_);
            ndt.setInputSource(cloud);
            ndt.align(align, T);
            double score = ndt.getFitnessScore();
            std::cout<<"score "<<score<<std::endl;
            result = T;
            return score < 3;
        }
    }

    bool GlobalLocalizationBBS::globalLocalization(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, const Eigen::Matrix4f& predict_pose, Eigen::Matrix4f& result){
        pcl::PointCloud<pcl::PointXYZI>::Ptr new_map(new pcl::PointCloud<pcl::PointXYZI>);
        // trans
        Eigen::Isometry3f center_t = Eigen::Isometry3f::Identity();
        center_t.translation() = -predict_pose.block<3, 1>(0, 3);
        pcl::transformPointCloud(*map_, *new_map, center_t);
        // filter
        float map_resolution, scan_resolution;
        nh_.getParam("bbs/map_filter_resolution", map_resolution);
        nh_.getParam("bbs/scan_filter_resolution", scan_resolution);
        //用ROS_INFO输出map_resolution和scan_resolution
//        ROS_INFO("111 map_resolution: %f", map_resolution);
//        ROS_INFO("111 scan_resolution: %f", scan_resolution);
//        map_resolution=0.05;
//        scan_resolution=0.05;
        new_map = downSample(new_map, map_resolution);
        auto filtered  = downSample(cloud, scan_resolution);
        // set match map
        set_match_map(new_map);
        // query
        auto re = query(filtered);
        re.sort(1);
        Eigen::Matrix4f T = re.results.at(0)->pose.matrix();
        T(2, 3) = 0;
        // match by ndt
        pcl::PointCloud<pcl::PointXYZI> align;
        ndt.setInputTarget(new_map);
        ndt.setInputSource(cloud);
        ndt.align(align, T);
        double score = ndt.getFitnessScore();
        std::cout<<"score "<<score<<std::endl;
        result = center_t.matrix().inverse() * T;
        return score < 3;
    }

    bool GlobalLocalizationBBS::globalLocalization(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, Eigen::Matrix4f& result){
        pcl::PointCloud<pcl::PointXYZI>::Ptr new_map(new pcl::PointCloud<pcl::PointXYZI>);
        // trans
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*map_, centroid);
        Eigen::Isometry3f center_t = Eigen::Isometry3f::Identity();
        center_t.translation() = -centroid.block<3, 1>(0, 0);
        pcl::transformPointCloud(*map_, *new_map, center_t);
        // filter
        float map_resolution, scan_resolution;
        nh_.getParam("bbs/map_filter_resolution", map_resolution);
        nh_.getParam("bbs/scan_filter_resolution", scan_resolution);
        new_map = downSample(new_map, map_resolution);
        auto filtered  = downSample(cloud, scan_resolution);
        // set match map
        {
            BBSParams params;
            double width, height;
            nh_.getParam("global_map_width", width);
            nh_.getParam("global_map_height", height);
            nh_.getParam("bbs/max_range", params.max_range);
            params.min_tx = -width/2;
            params.max_tx = width/2;
            params.min_ty = -height/2;
            params.max_ty = height/2;
            nh_.getParam("bbs/min_theta", params.min_theta);
            nh_.getParam("bbs/max_theta", params.max_theta);
            bbs.reset(new BBSLocalization(params));

            double map_min_z = -5.0, map_max_z = 15.0;
            nh_.getParam("bbs/map_min_z", map_min_z);
            nh_.getParam("bbs/map_max_z", map_max_z);
            auto map_2d = slice(*new_map, map_min_z, map_max_z);

            if (map_2d.size() < 128) {
                printf("Num points in the sliced map is too small!!");
                printf("Change the slice range parameters!!");
            }

            int map_width = 512, map_height = 1024, map_pyramid_level = 6, max_points_per_cell = 5;
            double map_resolution;
            nh_.getParam("bbs/map_width", map_width);
            nh_.getParam("bbs/map_height", map_height);
            nh_.getParam("bbs/map_resolution", map_resolution);
            nh_.getParam("bbs/map_pyramid_level", map_pyramid_level);
            nh_.getParam("bbs/max_points_per_cell", max_points_per_cell);

            bbs->set_map(map_2d, map_resolution, map_width, map_height, map_pyramid_level, max_points_per_cell);

            auto map_3d = unslice(map_2d);
            map_3d->header.frame_id = "map";
        }
        // query
        auto re = query(filtered);
        re.sort(1);
        if (re.results.empty())
            return false;

        Eigen::Matrix4f T = re.results.at(0)->pose.matrix();
        T(2, 3) = 0;
        // match by ndt
        pcl::PointCloud<pcl::PointXYZI> align;
        ndt.setInputTarget(new_map);
        ndt.setInputSource(cloud);
        ndt.align(align, T);
        double score = ndt.getFitnessScore();
        std::cout<<"score "<<score<<std::endl;
        result = center_t.matrix().inverse() * T;
        return score < 3;
    }

    void GlobalLocalizationBBS::set_match_map(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) {

        BBSParams params;
        nh_.getParam("bbs/max_range", params.max_range);
        nh_.getParam("bbs/min_tx", params.min_tx);
        nh_.getParam("bbs/max_tx", params.max_tx);
        nh_.getParam("bbs/min_ty", params.min_ty);
        nh_.getParam("bbs/max_ty", params.max_ty);
        nh_.getParam("bbs/min_theta", params.min_theta);
        nh_.getParam("bbs/max_theta", params.max_theta);
        bbs.reset(new BBSLocalization(params));

        double map_min_z = -5.0, map_max_z = 15.0;
        nh_.getParam("bbs/map_min_z", map_min_z);
        nh_.getParam("bbs/map_max_z", map_max_z);
        auto map_2d = slice(*cloud, map_min_z, map_max_z);

        if (map_2d.size() < 128) {
            printf("Num points in the sliced map is too small!!");
            printf("%d", map_min_z);
            printf("%d", map_max_z);

            printf("Change the slice range parameters!!");
        }

        int map_width = 512, map_height = 1024, map_pyramid_level = 6, max_points_per_cell = 5;
        double map_resolution;
        nh_.getParam("bbs/map_width", map_width);
        nh_.getParam("bbs/map_height", map_height);
        nh_.getParam("bbs/map_resolution", map_resolution);
        nh_.getParam("bbs/map_pyramid_level", map_pyramid_level);
        nh_.getParam("bbs/max_points_per_cell", max_points_per_cell);

        bbs->set_map(map_2d, map_resolution, map_width, map_height, map_pyramid_level, max_points_per_cell);

        auto map_3d = unslice(map_2d);
        map_3d->header.frame_id = "map";
    }

    GlobalLocalizationResults GlobalLocalizationBBS::query(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) {
        double scan_min_z = -1.5, scan_max_z = 1.5;
        nh_.getParam("bbs/scan_min_z", scan_min_z);
        nh_.getParam("bbs/scan_max_z", scan_max_z);
        auto scan_2d = slice(*cloud, scan_min_z, scan_max_z);

        std::vector<GlobalLocalizationResult::Ptr> results;

        if (scan_2d.size() < 32) {
            return GlobalLocalizationResults(results);
        }

        double best_score = 0.0;
        auto trans_2d = bbs->localize(scan_2d, 0.0, &best_score);
        if (trans_2d == boost::none) {
            return GlobalLocalizationResults(results);
        }

        Eigen::Isometry3f trans_3d = Eigen::Isometry3f::Identity();
        trans_3d.linear().block<2, 2>(0, 0) = trans_2d->linear();
        trans_3d.translation().head<2>() = trans_2d->translation();

        results.resize(1);
        results[0].reset(new GlobalLocalizationResult(best_score, best_score, trans_3d));

        return GlobalLocalizationResults(results);
    }

    GlobalLocalizationBBS::Points2D GlobalLocalizationBBS::slice(const pcl::PointCloud<pcl::PointXYZI>& cloud, double min_z, double max_z) const {
        Points2D points_2d;
        points_2d.reserve(cloud.size());
        for (auto i : cloud) {
            if (min_z < i.z && i.z < max_z) {
                points_2d.push_back(i.getVector3fMap().head<2>());
            }
        }
        return points_2d;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr GlobalLocalizationBBS::unslice(const Points2D& points) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud->resize(points.size());
        for (int i = 0; i < points.size(); i++) {
            cloud->at(i).getVector3fMap().head<2>() = points[i];
            cloud->at(i).z = 0.0f;
        }

        return cloud;
    }
}  // namespace global_localization
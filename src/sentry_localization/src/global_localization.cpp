#include <iostream>
#include <boost/filesystem.hpp>

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <sentry_userdefinition/SetGlobalMap.h>
#include <sentry_userdefinition/SetGlobalMapRequest.h>
#include <sentry_userdefinition/SetGlobalMapResponse.h>

#include <sentry_userdefinition/QueryGlobalLocalization.h>
#include <sentry_userdefinition/QueryGlobalLocalizationRequest.h>
#include <sentry_userdefinition/QueryGlobalLocalizationResponse.h>

#include <sentry_userdefinition/SetGlobalLocalizationEngine.h>
#include <sentry_userdefinition/SetGlobalLocalizationEngineRequest.h>
#include <sentry_userdefinition/SetGlobalLocalizationEngineResponse.h>


#include <include/std_cout.h>
#include <include/global_localization/engines/global_localization_bbs.hpp>
#include <include/global_localization/engines/global_localization_fpfh_ransac.hpp>
#include <include/global_localization/engines/global_localization_fpfh_teaser.hpp>

namespace sentry_localization
{
class GlobalLocalization   
{
    using PointT = pcl::PointXYZ;
public:
    GlobalLocalization() : nh(), private_nh("~"){
        setEngineName(private_nh.param<std::string>("global_localization_engine", "FPFH_RANSAC"));

        set_engine_server = private_nh.advertiseService("set_engine", &GlobalLocalization::set_engine, this);
        set_globalmap_server = private_nh.advertiseService("/sentry_userdefinition/set_globalmap", &GlobalLocalization::set_global_map, this);
        query_server = private_nh.advertiseService("/sentry_userdefinition/query", &GlobalLocalization::query, this);
    }

private:
    pcl::PointCloud<PointT>::Ptr downsample(pcl::PointCloud<PointT>::Ptr cloud, double size){
        pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
        pcl::ApproximateVoxelGrid<PointT> voxel;
        voxel.setLeafSize(size, size, size);
        voxel.setInputCloud(cloud);
        voxel.filter(*filtered_cloud);
        return filtered_cloud;
    }

    bool setEngineName(const std::string& engine_name){
        if(engine_name == "BBS"){
            engine.reset(new GlobalLocalizationBBS(private_nh));
        } else if(engine_name == "FPFH_RANSAC"){
            engine.reset(new GlobalLocalizationEngineFPFH_RANSAC(private_nh));
        }
        #ifdef TEASER_ENABLED
        else if(engine_name == "FPFH_TEASER"){
            engine.reset(new GlobalLocalizationEngineFPFH_Teaser(private_nh));
        }
        #endif
        else{
            std::cout << RED << "No correct global localization engine!" << RESET << std::endl;
            return false;
        }

        return true;
    }

    bool set_engine(sentry_userdefinition::SetGlobalLocalizationEngine::Request& req, 
                    sentry_userdefinition::SetGlobalLocalizationEngine::Response& res){
        std::cout << GREEN << "Set global localization engine!" << RESET << std::endl;
        return setEngineName(req.engine_name.data);
    }

    bool set_global_map(sentry_userdefinition::SetGlobalMap::Request& req, 
                        sentry_userdefinition::SetGlobalMap::Response& res){
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(req.global_map, *cloud);
        cloud = downsample(cloud, private_nh.param<double>("globalmap_downsample_resolution", 0.5));

        globalmap_header = req.global_map.header;
        globalmap = cloud;
        engine->set_global_map(globalmap);
        std::cout << GREEN << "Global map received and set done!" << RESET << std::endl;

        return true;
    }

    bool query(sentry_userdefinition::QueryGlobalLocalization::Request& req, 
               sentry_userdefinition::QueryGlobalLocalization::Response& res){
        if(globalmap == nullptr){
            std::cout << RED << "No global map received!" << RESET << std::endl;
            return false;
        }

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(req.cloud, *cloud);
        cloud = downsample(cloud, private_nh.param<double>("query_downsample_resolution", 0.5));
        auto results = engine->query(cloud, req.max_num_candidates);

        res.inlier_fractions.resize(results.results.size());
        res.errors.resize(results.results.size());
        res.poses.resize(results.results.size());
        res.header = req.cloud.header;
        res.globalmap_header = globalmap_header;

        for(int i = 0; i < results.results.size(); ++i){
            const auto& result = results.results[i];
            Eigen::Quaternionf quat(result->pose.linear());
            Eigen::Vector3f trans(result->pose.translation());

            res.inlier_fractions[i] = result->inlier_fraction;
            res.errors[i] = result->error;
            res.poses[i].position.x = trans.x();
            res.poses[i].position.y = trans.y();
            res.poses[i].position.z = trans.z();
            res.poses[i].orientation.x = quat.x();
            res.poses[i].orientation.y = quat.y();
            res.poses[i].orientation.z = quat.z();
            res.poses[i].orientation.w = quat.w();
        }
        return !results.results.empty();
    }

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::ServiceServer set_engine_server;
    ros::ServiceServer set_globalmap_server;
    ros::ServiceServer query_server;

    std_msgs::Header globalmap_header;
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalmap;
    std::unique_ptr<GlobalLocalizationEngine> engine;

};
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "global_localization_node");
    sentry_localization::GlobalLocalization global_localization_node;
    std::cout << GREEN << "Global localization start!" << RESET << std::endl;
    ros::spin(); 
    return 0;
}



















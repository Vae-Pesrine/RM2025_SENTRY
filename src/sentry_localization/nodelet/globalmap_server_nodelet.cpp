#include <mutex>
#include <memory>
#include <iostream>
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <include/std_cout.h>

namespace sentry_localization
{

class GlobalmapServerNodelet : public nodelet::Nodelet
{

public:
    GlobalmapServerNodelet() {}
    virtual ~GlobalmapServerNodelet() {}

    void onInit()  override
    {
        nh = getNodeHandle();
        mt_nh = getMTNodeHandle();
        pr_nh = getPrivateNodeHandle();

        InitParams();
        pub_globalmap = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
        sub_updatemap = nh.subscribe("/map_request/pcd", 10, &GlobalmapServerNodelet::globalmap_updateCb, this);
        //the first true创建定时器时开始计时      the second true 创建定时器时设置为自动重启
        pub_globalmap_timer = nh.createWallTimer(ros::WallDuration(1.0), &GlobalmapServerNodelet::globalmap_onceCb, this, true, true);
    }

private:
    void InitParams()
    {
        pr_nh.getParam("downsample_resolution", downsample_resolution);
        pr_nh.getParam("globalmap_pcdfile", globalmap_pcdfile);

        //reading pcd file
        globalmap_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        std::cout << BLUE << "Reading pcd file " << globalmap_pcdfile << RESET << std::endl;
        if(pcl::io::loadPCDFile(globalmap_pcdfile, *globalmap_cloud) == -1)
            PCL_ERROR("Could not read file");

        //voxel grid filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> voxelGridFilter(new pcl::VoxelGrid<pcl::PointXYZ>());
        voxelGridFilter->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        voxelGridFilter->setInputCloud(globalmap_cloud);
        voxelGridFilter->filter(*filtered_cloud);
        pcl::toROSMsg(*filtered_cloud, globalmap_roscloud);
        globalmap_roscloud.header.frame_id = "map";
        globalmap_cloud->clear();
        filtered_cloud->clear();
    }

    void globalmap_onceCb(const ros::WallTimerEvent& event)
    {
        pub_globalmap.publish(globalmap_roscloud);
    }

    void globalmap_updateCb(const std_msgs::String::ConstPtr msg)
    {
        std::cout << GREEN << "Received map request and the map path is " << msg->data << RESET << std::endl;
        globalmap_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if(pcl::io::loadPCDFile(msg->data, *globalmap_cloud) == -1)
            PCL_ERROR("Could not read pcd file");

        //voxel grid filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> voxelGridFilter(new pcl::VoxelGrid<pcl::PointXYZ>());
        voxelGridFilter->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        voxelGridFilter->setInputCloud(globalmap_cloud);
        voxelGridFilter->filter(*filtered_cloud);
        pcl::toROSMsg(*filtered_cloud, globalmap_roscloud);
        globalmap_roscloud.header.frame_id = "map";
        globalmap_cloud->clear();
        filtered_cloud->clear();
        globalmap_roscloud.header.frame_id = "map";
        pub_globalmap.publish(globalmap_roscloud);
    }

private:
    ros::NodeHandle nh;
    ros::NodeHandle mt_nh;
    ros::NodeHandle pr_nh;
    ros::Publisher pub_globalmap;
    ros::Subscriber sub_updatemap;
    ros::WallTimer pub_globalmap_timer;

    std::string globalmap_pcdfile;
    double downsample_resolution;
    sensor_msgs::PointCloud2 globalmap_roscloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr globalmap_cloud;

};

}

PLUGINLIB_EXPORT_CLASS(sentry_localization::GlobalmapServerNodelet, nodelet::Nodelet)
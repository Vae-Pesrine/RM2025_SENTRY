//my header
#include "pcd_sdf/pcl_process.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pcd_pub;
ros::Timer estimate_update_timer;
ros::Subscriber cloud_sub;

PclProcess pcl_process;

void pcdLoadCallback(const ros::TimerEvent&){
  ROS_INFO("PCD file is Load");
  PointCloud::Ptr transformed_cloud(new PointCloud);
  pcl_process.loadPCD(transformed_cloud);
  // Pub the point cloud
  PointCloud cloud = *transformed_cloud;
  cloud.header.frame_id = "world";
  pcd_pub.publish(cloud);
}

void publisherInit(ros::NodeHandle& nh)
{
  pcd_pub = nh.advertise<PointCloud>("/pcd", 1, false);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_2_gridmap");

  ros::NodeHandle nh;

  estimate_update_timer = nh.createTimer(ros::Duration(1.0), pcdLoadCallback);

  publisherInit(nh);

  ros::spin();
}
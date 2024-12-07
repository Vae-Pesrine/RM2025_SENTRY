#ifndef ROBOTICS_SDF_PCL_PROCESS_HPP
#define ROBOTICS_SDF_PCL_PROCESS_HPP

#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/vtk_io.h>


class PclProcess{
public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  //Initialization
  PclProcess();
  ~PclProcess();

  bool loadPCD(PointCloud::Ptr &output);
};

// Definition
//-------------------------------------------------------------------------------------------------
PclProcess::PclProcess() {}
PclProcess::~PclProcess() {}

bool PclProcess::loadPCD(PointCloud::Ptr &output){
  PointCloud::Ptr cloud_load(new PointCloud);


  std::string file_name;
  std::string r;

  ros::param::search("name", r);
  ros::param::get(r, file_name);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud_load) == -1){
    PCL_ERROR("Couldn't read file\n");
  }

//  // Compute the center pos
//  pcl::PointXYZ minPt, maxPt;
//  pcl::getMinMax3D(*cloud_load, minPt, maxPt);
//  float center_x = (maxPt.x - minPt.x)/2;
//  float center_y = (maxPt.y - minPt.y)/2;
//  float center_z = (maxPt.z - minPt.z)/2;
  // Transform the point cloud
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.translation() << 0,0,0;
  transform_2.rotate (Eigen::AngleAxisf (0.0, Eigen::Vector3f::UnitZ()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*cloud_load, *transformed_cloud, transform_2);
  // Pub the point cloud
  *output = *transformed_cloud;

  return true;
}


#endif //ROBOTICS_SDF_PCL_PROCESS_HPP

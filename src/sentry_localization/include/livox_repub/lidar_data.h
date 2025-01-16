#ifndef LIDAR_DATA_H
#define LIDAR_DATA_H

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>

struct PointXYZIRT {
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float intensity;                 ///< laser intensity reading
  uint16_t ring;                   ///< laser ring number
  float time;                      ///< laser time reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;


POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, 
                                  (float, x, x)  //
                                  (float, y, y)                             //
                                  (float, z, z)                             //
                                  (float, intensity, intensity)             //
                                  (uint16_t, ring, ring)                    //
                                  (float, time, time)                       //
)

typedef PointXYZIRT RTPoint;
typedef pcl::PointCloud<RTPoint> RTPointCloud;

#endif
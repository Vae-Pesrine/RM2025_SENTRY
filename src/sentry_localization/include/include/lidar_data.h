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

// the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */


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
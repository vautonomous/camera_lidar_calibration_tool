#ifndef SRC_POINT_CLOUD_ALIGNER_INCLUDE_POINTCLOUDHANDLER_H_
#define SRC_POINT_CLOUD_ALIGNER_INCLUDE_POINTCLOUDHANDLER_H_

#include <ros/ros.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>



#include <ros/ros.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/impl/project_inliers.hpp>

class PointCloudHandler
{
 public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;




  static Cloud::Ptr
  Points2Plane(const Cloud::Ptr& cloud,
               Eigen::Vector4d& vector_normal_coeff,
               Eigen::Vector4f& plane_coeff,
               Point& middle_point);
};

#endif  // SRC_POINT_CLOUD_ALIGNER_INCLUDE_POINTCLOUDHANDLER_H_

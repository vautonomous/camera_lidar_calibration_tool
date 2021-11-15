#ifndef SRC_EXTRINSIC_CALIBRATOR_INCLUDE_SAMPLE_H_
#define SRC_EXTRINSIC_CALIBRATOR_INCLUDE_SAMPLE_H_

#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv4/opencv2/highgui.hpp>

class Sample
{
public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;

  cv::Mat image;
  Cloud::Ptr cloud;

  Cloud::Ptr cloud_plane_points_lidar;
  Eigen::Vector3d normal_vector_lid;
  Point middle_point_lid;

  Cloud::Ptr cloud_plane_points_cam;
  Eigen::Vector3d normal_vector_cam;
  std::vector<Eigen::Vector4d> vector_3d_points_in_cam_frame;
  Eigen::Vector4d plane_coeff_cam;
  Point middle_point_cam;

  Cloud::Ptr cloud_test;
  Cloud::Ptr cloud_frustum;
  Cloud::Ptr cloud_frustum_colored;

  bool is_feature_extracted;

  Sample() : cloud(new Cloud), cloud_plane_points_lidar(new Cloud),
  cloud_plane_points_cam(new Cloud), cloud_test(new Cloud),
  cloud_frustum(new Cloud), cloud_frustum_colored(new Cloud), is_feature_extracted(false) {}

  void set_image(const cv::Mat &input_image) {input_image.copyTo(image);}
  void set_cloud(const Cloud::Ptr &input_cloud) {pcl::copyPointCloud(*input_cloud, *cloud);}
  void set_test_cloud(const Cloud::Ptr &input_cloud) {pcl::copyPointCloud(*input_cloud, *cloud_test);}

};
#endif  // SRC_EXTRINSIC_CALIBRATOR_INCLUDE_SAMPLE_H_

#ifndef SRC_EXTRINSIC_CALIBRATOR_INCLUDE_FEATURE_EXTRACTOR_H_
#define SRC_EXTRINSIC_CALIBRATOR_INCLUDE_FEATURE_EXTRACTOR_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sample.h>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <ros_handler.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <point_cloud_handler.h>
#include <calibration_parameters.h>


using namespace std::chrono;

class FeatureExtractor
{
 public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;

  static Cloud::Ptr
  extract_image_features(Sample& sample,
                         const CalibrationParameters& calibration_parameters,
                         const ros::Publisher& pub_plane_points_cam);

  static Cloud::Ptr
  extract_lidar_features(Sample& sample, const Cloud::Ptr& cloud_selected_points,
                         const ros::Publisher& pub_plane_points_lid);

};

#endif  // SRC_EXTRINSIC_CALIBRATOR_INCLUDE_FEATURE_EXTRACTOR_H_

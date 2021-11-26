#ifndef SRC_EXTRINSIC_CALIBRATOR_INCLUDE_TEST_H_
#define SRC_EXTRINSIC_CALIBRATOR_INCLUDE_TEST_H_

#include <calibration_parameters.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <sample.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/transforms.h>


class Test
{
 public:

  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;

  static void project_lidar_points_to_image_plane(Sample& sample,
                                                  const CalibrationParameters& calibration_parameters,
                                                  const Eigen::Affine3d tf,
                                                  const image_transport::Publisher& pub_undistorted_image_,
                                                  const ros::Publisher& pub_cloud_frustum_colored,
                                                  const double& max_distance_cam_seen);

  static std::tuple<int, int, int> distance_to_color(const double& distance,
                                                    const double& max_distance);

  static pcl::PointXYZRGB
  giveColoredPoint(const cv::Mat &image, cv::Point &point_in_image,
                         const Point &cloud_point);



};

#endif  // SRC_EXTRINSIC_CALIBRATOR_INCLUDE_TEST_H_

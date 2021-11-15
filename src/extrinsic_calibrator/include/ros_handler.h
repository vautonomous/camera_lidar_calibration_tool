#ifndef SRC_EXTRINSIC_CALIBRATOR_INCLUDE_ROS_HANDLER_H_
#define SRC_EXTRINSIC_CALIBRATOR_INCLUDE_ROS_HANDLER_H_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class ROSHandler
{
public:

  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;

  static void
  PublishCloud(const ros::Publisher &publisher,
              const Cloud::Ptr &cloud,
              const std::string &frame_id);

  static void
  PublishImage(const image_transport::Publisher& pub_image_,
               const cv::Mat& image,
               const std::string& frame_id);

  static void
  PutArrowMarker(
      const Eigen::Vector3d& normal_coeff,
      const Point& middle_point,
      visualization_msgs::MarkerArray& markerArray_plane_normals,
      int& marker_id);

};

#endif  // SRC_EXTRINSIC_CALIBRATOR_INCLUDE_ROS_HANDLER_H_

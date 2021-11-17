
#include <opencv4/opencv2/highgui.hpp>
#include <ros/ros.h>
#include <point_cloud_handler.h>
#include <ros/package.h>

#include <std_msgs/Int32.h>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <thread>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <data_write_read.h>
#include <ros_handler.h>
#include <feature_extractor.h>
#include <sample.h>
#include <rotation_solver.h>
#include <pcl/io/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include "calibration_parameters.h"
#include "test.h"
#include <translation_solver.h>


class PointCloudAligner {
 public:

  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;

  ros::NodeHandle &nh_;
  image_transport::ImageTransport it;
  explicit PointCloudAligner(ros::NodeHandle &nh);

  ros::Timer timer_pc_publisher_;
  ros::Subscriber sub_key_;
  ros::Subscriber sub_selected_points_;
  void CallbackSamplePublisher(const ros::TimerEvent &);
  void KeyCallback(const std_msgs::Int32::ConstPtr &msg);
  void CallbackClickedPoint(const sensor_msgs::PointCloud2ConstPtr &msg_cloud_selected_points);

  size_t mode;
  size_t sample_id;
  size_t total_sample_count;

  ros::Publisher pub_cloud_;
  ros::Publisher pub_cloud_plane_points_cam_;
  ros::Publisher pub_cloud_plane_points_lid_;
  ros::Publisher pub_cloud_frustum_colored;
  image_transport::Publisher pub_image_;
  image_transport::Publisher pub_undistorted_image_;
  ros::Publisher pub_marker_array_;
  visualization_msgs::MarkerArray marker_array_;
  int marker_id_ = 0;

  std::vector<Sample> vector_samples;

  Cloud::Ptr cloud_selected_points;

  bool is_rot_optim;
  Eigen::Affine3d trans_optimized;

  std::vector<Cloud::Ptr> vector_all_planes_cam;
  std::vector<Cloud::Ptr> vector_all_planes_lid;

  CalibrationParameters calibration_parameters_;

};


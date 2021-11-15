#include "test.h"

void Test::project_lidar_points_to_image_plane(Sample& sample,
                                               const CalibrationParameters& calibration_parameters,
                                               const Eigen::Affine3d tf,
                                               const image_transport::Publisher& pub_undistorted_image_,
                                               const ros::Publisher& pub_cloud_frustum_colored)
{
  cv::Mat undistortedImage;
  cv::undistort(sample.image, undistortedImage,
                calibration_parameters.cameramat,
                calibration_parameters.distcoeff);
  // Extrinsic:
  Eigen::Matrix4d mat_transf_lid_to_cam = Eigen::MatrixXd::Identity(4,4);
  mat_transf_lid_to_cam.topLeftCorner(3, 3) = tf.linear().topLeftCorner(3, 3);
  mat_transf_lid_to_cam(0, 3) = tf.translation().x();
  mat_transf_lid_to_cam(1, 3) = tf.translation().y();
  mat_transf_lid_to_cam(2, 3) = tf.translation().z();

  // Intrinsic:
  Eigen::Matrix3d mat_cam_intrinsic_3x3 = Eigen::MatrixXd::Zero(3,3);
  mat_cam_intrinsic_3x3 <<
  calibration_parameters.cameramat.at<double>(0),
  calibration_parameters.cameramat.at<double>(1),
  calibration_parameters.cameramat.at<double>(2),
  calibration_parameters.cameramat.at<double>(3),
  calibration_parameters.cameramat.at<double>(4),
  calibration_parameters.cameramat.at<double>(5),
  calibration_parameters.cameramat.at<double>(6),
  calibration_parameters.cameramat.at<double>(7),
  calibration_parameters.cameramat.at<double>(8);

  // Mat point transformer:
  Eigen::Matrix4d mat_point_projector = Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd mat(3,4);
  mat = mat_cam_intrinsic_3x3*mat_transf_lid_to_cam.topLeftCorner(3,4);
  mat_point_projector.topLeftCorner(3,4) = mat;

  Cloud::Ptr frustum(new Cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr frustum_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

  cv::Mat undistorted_image_clear;
  undistortedImage.copyTo(undistorted_image_clear);

  for (const auto &point : sample.cloud_test->points)
  {
    Eigen::Vector4d pos;
    pos << point.x, point.y, point.z, 1;

    Eigen::Vector4d vec_image_plane_coords = mat_point_projector * pos;

    if (vec_image_plane_coords(2) <= 0)
      continue;

    cv::Point point_in_image;

    point_in_image.x = (int) (vec_image_plane_coords(0) / vec_image_plane_coords(2));
    point_in_image.y = (int) (vec_image_plane_coords(1) / vec_image_plane_coords(2));

    if (point_in_image.x < 0
        || point_in_image.y < 0
        || point_in_image.x >= sample.image.cols
        || point_in_image.y >= sample.image.rows)
      continue;

    frustum->points.push_back(point);

    pcl::PointXYZRGB point_c =  Test::giveColoredPoint(undistorted_image_clear, point_in_image, point);
    frustum_colored->points.push_back(point_c);

    auto getDistance = [](const Point& p)->float
    {
      float distance = sqrt(pow(p.x,2) + pow(p.y,2));
      return distance;
    };
    float dist = getDistance(point);
    auto color = distance_to_color(dist, 30);
    int r = std::get<0>(color);
    int g = std::get<1>(color);
    int b = std::get<2>(color);

    int radiusCircle = 1;
    cv::Scalar colorCircle1(r,g,b);
    int thicknessCircle1 = 4;

    cv::circle(undistortedImage, point_in_image, radiusCircle, colorCircle1, thicknessCircle1);
  }

  pcl::transformPointCloud(*frustum_colored, *frustum_colored, tf);
  // Publish frustum colored cloud:
  sensor_msgs::PointCloud2 msg_cloud_frustum_colored;
  pcl::toROSMsg(*frustum_colored, msg_cloud_frustum_colored);
  msg_cloud_frustum_colored.header.frame_id = "camera_frame";
  msg_cloud_frustum_colored.header.stamp = ros::Time::now();
  pub_cloud_frustum_colored.publish(msg_cloud_frustum_colored);

  // Publish undistorted image:
  cv_bridge::CvImage out_msg;
  out_msg.header.frame_id   = "camera_frame";
  out_msg.header.stamp = ros::Time::now();
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image    = undistortedImage;
  pub_undistorted_image_.publish(out_msg.toImageMsg());


}



std::tuple<int, int, int> Test::distance_to_color(const double& distance,
                                                  const double& max_distance) {
  double hh, p, q, t, ff;
  long i;
  double v = 0.75;
  double s = 0.75;
  double r, g, b;
  double h = ((max_distance - distance) / max_distance) * 360.0;
  hh = h;
  if (hh >= 360.0) hh = 0.0;
  hh /= 60.0;
  i = (long) hh;
  ff = hh - i;
  p = v * (1.0 - s);
  q = v * (1.0 - (s * ff));
  t = v * (1.0 - (s * (1.0 - ff)));

  switch (i) {
    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;

    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    case 5:
    default:
      r = v;
      g = p;
      b = q;
      break;
  }
  return std::make_tuple((int) (r * 255), (int) (g * 255), (int) (b * 255));

}

pcl::PointXYZRGB
Test::giveColoredPoint(const cv::Mat &image, cv::Point &point_in_image,
                       const Point &cloud_point)
{
  const cv::Vec3b& color_of_point = image.at<cv::Vec3b>(point_in_image);
  pcl::PointXYZRGB point_colored = pcl::PointXYZRGB(color_of_point[2],
                                                    color_of_point[1],
                                                    color_of_point[0]);
  point_colored.x = cloud_point.x;
  point_colored.y = cloud_point.y;
  point_colored.z = cloud_point.z;
  return point_colored;
}

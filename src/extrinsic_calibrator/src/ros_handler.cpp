#include "ros_handler.h"
void
ROSHandler::PublishCloud(const ros::Publisher &publisher,
                         const Cloud::Ptr &cloud,
                         const std::string &frame_id) {
  cloud->width = cloud->points.size();
  cloud->height = 1;
  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*cloud, msg_cloud);
  msg_cloud.header.frame_id = frame_id;
  msg_cloud.header.stamp = ros::Time::now();
  publisher.publish(msg_cloud);
}
void ROSHandler::PublishImage(const image_transport::Publisher &pub_image_,
                              const cv::Mat &image,
                              const std::string &frame_id)
{
  cv_bridge::CvImage out_msg;
  out_msg.header.frame_id   = frame_id;
  out_msg.header.stamp = ros::Time::now();
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image    = image;
  pub_image_.publish(out_msg.toImageMsg());
}


void
ROSHandler::PutArrowMarker(
    const Eigen::Vector3d& normal_coeff,
    const Point& middle_point,
    visualization_msgs::MarkerArray& markerArray_plane_normals,
    int& marker_id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "camera_frame";
  marker.header.stamp = ros::Time();
  marker.ns = "arrow";
  marker.id = marker_id;
  marker_id++;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.02;
  marker.scale.y = 0.04;
  marker.scale.z = 0.06;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  geometry_msgs::Point start, end;
  start.x = middle_point.x;
  start.y = middle_point.y;
  start.z = middle_point.z;
  end.x = start.x + normal_coeff(0)/2;
  end.y = start.y + normal_coeff(1)/2;
  end.z = start.z + normal_coeff(2)/2;
  marker.points.resize(2);
  marker.points[0].x = start.x;
  marker.points[0].y = start.y;
  marker.points[0].z = start.z;
  marker.points[1].x = end.x;
  marker.points[1].y = end.y;
  marker.points[1].z = end.z;

  markerArray_plane_normals.markers.push_back(marker);
}


#include "point_cloud_handler.h"

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;



Cloud::Ptr
PointCloudHandler::Points2Plane(
    const Cloud::Ptr& cloud,
    Eigen::Vector4d& vector_normal_coeff,
    Eigen::Vector4f& plane_coeff,
    Point& middle_point)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::SACSegmentation<Point> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.2);
  pcl::ExtractIndices<Point> extract;
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  plane_coeff(0) = coefficients->values[0];
  plane_coeff(1) = coefficients->values[1];
  plane_coeff(2) = coefficients->values[2];
  plane_coeff(3) = coefficients->values[3];

  float mag = sqrt(pow(coefficients->values[0], 2)
                       + pow(coefficients->values[1], 2)
                       + pow(coefficients->values[2], 2));

  vector_normal_coeff(0) = -coefficients->values[0] / mag;
  vector_normal_coeff(1) = -coefficients->values[1] / mag;
  vector_normal_coeff(2) = -coefficients->values[2] / mag;
  vector_normal_coeff(3) = 1;

  // Determine min max points:
  std::vector<float> x_values;
  std::vector<float> y_values;
  std::vector<float> z_values;
  for(const auto& point : cloud->points)
  {
    x_values.push_back(point.x);
    y_values.push_back(point.y);
    z_values.push_back(point.z);
  }
  float min_x = *std::min_element(x_values.begin(), x_values.end());
  float max_x = *std::max_element(x_values.begin(), x_values.end());
  float min_y = *std::min_element(y_values.begin(), y_values.end());
  float max_y = *std::max_element(y_values.begin(), y_values.end());
  float min_z = *std::min_element(z_values.begin(), z_values.end());
  float max_z = *std::max_element(z_values.begin(), z_values.end());
  float epsilon = 0.1;

  Cloud::Ptr cloud_random_points(new Cloud);

  float x_temp;
  float y_temp;
  float z_temp;

  //std::cout << min_x << " " << max_x << " " << min_y << " " << max_y << " " << min_z << " " << max_z << std::endl;

  x_temp = min_x;
  while(x_temp < max_x)
  {
    y_temp = min_y;
    while(y_temp < max_y)
    {
      z_temp = min_z;
      while(z_temp < max_z)
      {
        Point point;
        point.x = x_temp;
        point.y = y_temp;
        point.z = z_temp;
        cloud_random_points->points.push_back(point);
        z_temp += epsilon;
      }
      y_temp += epsilon;
    }
    x_temp += epsilon;
  }

  Cloud::Ptr cloud_projected(new Cloud);
  pcl::ProjectInliers<Point> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud_random_points);
  proj.setModelCoefficients(coefficients);
  proj.filter(*cloud_projected);

  float sum_x = 0;
  float sum_y = 0;
  float sum_z = 0;

  for(const auto p:cloud_projected->points)
  {
    sum_x += p.x;
    sum_y += p.y;
    sum_z += p.z;
  }

  middle_point.x = sum_x/cloud_projected->points.size();
  middle_point.y = sum_y/cloud_projected->points.size();
  middle_point.z = sum_z/cloud_projected->points.size();

  /* std::cout << "Middle point: " << middle_point.x << " " <<
   middle_point.y << " " << middle_point.z << std::endl;*/

  return cloud_projected;
}

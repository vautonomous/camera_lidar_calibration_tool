#include "feature_extractor.h"

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

Cloud::Ptr FeatureExtractor::extract_image_features(Sample& sample,
                                                    const CalibrationParameters& calibration_parameters,
                                                    const ros::Publisher& pub_plane_points_cam)
{
  cv::Mat distcoeff = calibration_parameters.distcoeff;
  cv::Mat cameramat = calibration_parameters.cameramat;
  cv::Size2i patternNum = calibration_parameters.patternNum;
  cv::Size2i patternSize = calibration_parameters.patternSize;

  cv::Mat gray;
  std::vector<cv::Point2f> corners;
  cv::cvtColor(sample.image, gray, cv::COLOR_BGR2GRAY);

  // Find checkerboard pattern in the image plane
  auto start = high_resolution_clock::now();
  bool patternfound = cv::findChessboardCorners(gray, patternNum, corners,
                                                cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(stop - start);

 /* std::cout << "Time taken by findChessboardCorners: "
            << duration.count() << " milliseconds" << std::endl;*/

  if (patternfound) {
    // Refinement:
    cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                 cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

    // Draw the checkerboard pattern with red color:
    for (size_t i = 0; i < corners.size(); i++) {
      int radius = 1;
      cv::Scalar color(0, 0, 255); // bgr order
      int thickness = 12;
      cv::Point checkerboard_corner = corners.at(i);
      //cv::circle(sample.image, checkerboard_corner, radius, color, thickness);
    }

    // Define the 3d points in board frame:
    std::vector<cv::Point3f> points_3d_in_board_frame;
    double tx, ty;
    tx = (patternNum.height - 1) * patternSize.height / 2;
    ty = (patternNum.width - 1) * patternSize.width / 2;
    for (int i = 0; i < patternNum.height; i++) {
      for (int j = 0; j < patternNum.width; j++) {
        cv::Point3f tmpgrid3dpoint;
        // Translating origin from bottom left corner to the centre of the checkerboard
        tmpgrid3dpoint.x = i * patternSize.height - tx;
        tmpgrid3dpoint.y = j * patternSize.width - ty;
        tmpgrid3dpoint.z = 0;
        points_3d_in_board_frame.push_back(tmpgrid3dpoint);

      }
    }

    /*
     * Use 3d points in the board frame
     * with PnP algorithm to get board
     * to camera frame transformation
     */


    // Pnp: Find transform between board and camera frame
    cv::Mat rvec(3, 3, cv::DataType<double>::type);
    cv::Mat tvec(3, 1, cv::DataType<double>::type);
    std::vector<cv::Point2f> image_points;
    cv::solvePnPRansac(points_3d_in_board_frame,
                       corners,
                       cameramat,
                       distcoeff, rvec, tvec);
    cv::projectPoints(points_3d_in_board_frame,
                      rvec, tvec, cameramat,
                      distcoeff, image_points);

    // Draw re-projected points with green color:
    for (size_t i = 0; i < image_points.size(); i++) {
      int radius = 1;
      cv::Scalar color(0, 255, 0); // bgr order
      int thickness = 12;
      cv::Point checkerboard_corner = corners.at(i);
      //cv::circle(sample.image, image_points[i], radius, color, thickness);
    }

    cv::Mat cv_mat_trans_board_to_cam = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat tmprmat = cv::Mat(3, 3, CV_64F); // rotation matrix
    cv::Rodrigues(rvec, tmprmat); // Euler angles to rotation matrix

    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        cv_mat_trans_board_to_cam.at<double>(j, k) = tmprmat.at<double>(j, k);
      }
      cv_mat_trans_board_to_cam.at<double>(j, 3) = tvec.at<double>(j) / 1000; // scale mm to m
    }

    Eigen::Matrix4d eigen_mat_trans_board_to_cam = Eigen::MatrixXd::Identity(4,4);
    for (size_t i = 0; i < 4; i++) {
      for (size_t j = 0; j < 4; j++) {
        eigen_mat_trans_board_to_cam(i, j) = cv_mat_trans_board_to_cam.at<double>(i, j);
      }
    }
    Eigen::Matrix3d rot_board_to_cam = eigen_mat_trans_board_to_cam.topLeftCorner(3,3);
    Eigen::Quaterniond q_board_to_cam(rot_board_to_cam);

    std::vector<Eigen::Vector4d> vector_3d_points_in_cam_frame;
    Cloud::Ptr cloud_3d_points_in_cam_frame(new Cloud);
    for (size_t i=0; i<points_3d_in_board_frame.size(); i++) {
      auto point = points_3d_in_board_frame.at(i);
      Eigen::Vector4d point_3d_in_board_frame = {point.x/1000, point.y/1000, point.z/1000, 1};
      Eigen::Vector4d point_3d_in_cam_frame = eigen_mat_trans_board_to_cam*point_3d_in_board_frame;
      vector_3d_points_in_cam_frame.push_back(point_3d_in_cam_frame);
      Point point_in_cloud;
      point_in_cloud.x = point_3d_in_cam_frame(0);
      point_in_cloud.y = point_3d_in_cam_frame(1);
      point_in_cloud.z = point_3d_in_cam_frame(2);
      cloud_3d_points_in_cam_frame->points.push_back(point_in_cloud);
    }

    sample.vector_3d_points_in_cam_frame = vector_3d_points_in_cam_frame;
    sample.cloud_plane_points_cam = cloud_3d_points_in_cam_frame;


    // Fit a plane through the board point cloud
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<Point> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.05);
    pcl::ExtractIndices<Point> extract;
    seg.setInputCloud(sample.cloud_plane_points_cam);
    seg.segment(*inliers, *coefficients);

    pcl::ProjectInliers<Point> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(sample.cloud_plane_points_cam);
    proj.setModelCoefficients(coefficients);
    proj.filter(*sample.cloud_plane_points_cam);

    float mag = sqrt(pow(coefficients->values[0], 2)
                         + pow(coefficients->values[1], 2)
                         + pow(coefficients->values[2], 2));

    sample.normal_vector_cam(0) = -coefficients->values[0] / mag;
    sample.normal_vector_cam(1)  = -coefficients->values[1] / mag;
    sample.normal_vector_cam(2)  = -coefficients->values[2] / mag;


    ROSHandler::PublishCloud(pub_plane_points_cam, sample.cloud_plane_points_cam, "camera_frame");

    /*std::cout << "Plane coefficients in the camera frame: " << coefficients->values[0] << " " <<
    coefficients->values[1] << " " << coefficients->values[2] <<
    " " << coefficients->values[3] << std::endl;*/

    for (int i = 0; i < 4; ++i)
      sample.plane_coeff_cam(i) = coefficients->values[i];


    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_frame";
    transformStamped.child_frame_id = "board_frame";
    transformStamped.transform.translation.x = eigen_mat_trans_board_to_cam(0,3);
    transformStamped.transform.translation.y = eigen_mat_trans_board_to_cam(1,3);
    transformStamped.transform.translation.z = eigen_mat_trans_board_to_cam(2,3);
    tf2::Quaternion q;
    transformStamped.transform.rotation.x = q_board_to_cam.x();
    transformStamped.transform.rotation.y = q_board_to_cam.y();
    transformStamped.transform.rotation.z = q_board_to_cam.z();
    transformStamped.transform.rotation.w = q_board_to_cam.w();
    br.sendTransform(transformStamped);

    float sum_x = 0;
    float sum_y = 0;
    float sum_z= 0;

    for (const auto& point : sample.cloud_plane_points_cam->points)
    {
      sum_x = sum_x + point.x;
      sum_y = sum_y + point.y;
      sum_z = sum_z + point.z;
    }
    sample.middle_point_cam.x = sum_x/sample.cloud_plane_points_cam->points.size();
    sample.middle_point_cam.y = sum_y/sample.cloud_plane_points_cam->points.size();
    sample.middle_point_cam.z = sum_z/sample.cloud_plane_points_cam->points.size();

    return sample.cloud_plane_points_cam;


    // EIGEN ###########################################################################################################
    // Make eigen to implement leo type projection:
/*    Eigen::MatrixXd eigen_mat_trans_board_to_cam(3,4);
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 4; j++) {
        eigen_mat_trans_board_to_cam(i, j) = cv_mat_trans_board_to_cam.at<double>(i, j);
      }
    }
    std::cout << "Transformation matrix board frame to camera frame:\n" << eigen_mat_trans_board_to_cam << std::endl;
    Eigen::MatrixXd eigen_mat_intrinsic(3,3);
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        eigen_mat_intrinsic(i, j) = cameramat.at<double>(i, j);
      }
    }
    std::cout << "Intrinsic matrix:\n" << eigen_mat_intrinsic << std::endl;
    Eigen::Matrix4d mat_point_transformer = Eigen::Matrix4d::Identity();
    mat_point_transformer.topLeftCorner(3,4) = eigen_mat_intrinsic*eigen_mat_trans_board_to_cam;

    // Transform 3d points board frame to camera frame:
    cv::Mat undistortedImage;
    cv::undistort(sample.image, undistortedImage, cameramat, distcoeff);
    sample.image = undistortedImage;
    for (size_t i = 0; i < points_3d_in_board_frame.size(); i++) {
      auto point = points_3d_in_board_frame.at(i);
      Eigen::Vector4d point_3d_in_board_frame = {point.x/1000, point.y/1000, point.z/1000, 1};
      Eigen::Vector4d point_3d_in_cam_frame = mat_point_transformer*point_3d_in_board_frame;
      if (point_3d_in_cam_frame(2) <= 0)
        continue;
      cv::Point point_in_image;
      point_in_image.x = (int) (point_3d_in_cam_frame(0) / point_3d_in_cam_frame(2));
      point_in_image.y = (int) (point_3d_in_cam_frame(1) / point_3d_in_cam_frame(2));
      std::cout << point_in_image.y << " " << point_in_image.x << std::endl;
      int radius = 1;
      cv::Scalar color(255, 0, 0); // bgr order
      int thickness = 12;
      cv::Point checkerboard_corner = corners.at(i);
      cv::circle(undistortedImage, point_in_image, radius, color, thickness);
    }*/
    // EIGEN END


  } // bool patternfound
  else {
    Cloud::Ptr cloud_empty(new Cloud);
    return cloud_empty;
  }


}

Cloud::Ptr
FeatureExtractor::extract_lidar_features(Sample &sample,
                                              const Cloud::Ptr &cloud_selected_points,
                                              const ros::Publisher &pub_plane_points_lid)
{
  Eigen::Vector4d normal_coeff_source;
  Eigen::Vector4f plane_coeff_source;
  Point middle_point_source;
  Cloud::Ptr cloud_plane_source = PointCloudHandler::Points2Plane(cloud_selected_points,
                                                                  normal_coeff_source,
                                                                  plane_coeff_source,
                                                                  middle_point_source);

  float sum_x = 0;
  float sum_y = 0;
  float sum_z= 0;

  for (const auto& point : cloud_plane_source->points)
  {
    sum_x = sum_x + point.x;
    sum_y = sum_y + point.y;
    sum_z = sum_z + point.z;
  }
  sample.middle_point_lid.x = sum_x/cloud_plane_source->points.size();
  sample.middle_point_lid.y = sum_y/cloud_plane_source->points.size();
  sample.middle_point_lid.z = sum_z/cloud_plane_source->points.size();


  sample.normal_vector_lid(0) = normal_coeff_source(0);
  sample.normal_vector_lid(1) = normal_coeff_source(1);
  sample.normal_vector_lid(2) = normal_coeff_source(2);

  double top_down_radius = sqrt(pow(sample.middle_point_lid.x, 2)
                                    + pow(sample.middle_point_lid.y, 2));
  double x_comp = sample.middle_point_lid.x + sample.normal_vector_lid(0)/ 2;
  double y_comp = sample.middle_point_lid.y + sample.normal_vector_lid(1) / 2;
  double vector_dist = sqrt(pow(x_comp, 2) + pow(y_comp, 2));
  if (vector_dist > top_down_radius) {
    sample.normal_vector_lid(0) = -sample.normal_vector_lid(0);
    sample.normal_vector_lid(1) = -sample.normal_vector_lid(1);
    sample.normal_vector_lid(2) = -sample.normal_vector_lid(2);
  }

  sample.cloud_plane_points_lidar = cloud_plane_source;

  ROSHandler::PublishCloud(pub_plane_points_lid, cloud_plane_source, "camera_frame");

  return sample.cloud_plane_points_lidar;
}





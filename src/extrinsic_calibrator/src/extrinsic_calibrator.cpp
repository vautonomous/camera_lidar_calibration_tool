
#include "extrinsic_calibrator.h"

PointCloudAligner::PointCloudAligner(ros::NodeHandle &nh)
    :
    nh_(nh),
    it(nh_),
    sample_id(0),
    total_sample_count(0),
    cloud_selected_points(new Cloud),
    is_rot_optim(false),
    mode(0) {

  dist_coeff.resize(5);
  camera_mat.resize(9);

  nh_.getParam("mode",mode);  nh_.getParam("distortion_coefficients",dist_coeff);
  nh_.getParam("camera_matrix",camera_mat);
  nh_.getParam("pattern_num_width",pattern_num_width);
  nh_.getParam("pattern_num_height",pattern_num_height);
  nh_.getParam("pattern_size_mm",pattern_size_mm);
  nh_.getParam("q_optimized",vector_q_param);
  nh_.getParam("trans_optimized",vector_trans_param);
  nh_.getParam("max_distance_cam_seen",max_distance_cam_seen);

  // ##########################################################################################

  cv::Mat(1, 5, CV_64F, &dist_coeff[0]).copyTo(calibration_parameters_.distcoeff);
  cv::Mat(3, 3, CV_64F, &camera_mat[0]).copyTo(calibration_parameters_.cameramat);
  calibration_parameters_.patternNum.width = pattern_num_width;
  calibration_parameters_.patternNum.height = pattern_num_height;
  calibration_parameters_.patternSize.width = pattern_size_mm;
  calibration_parameters_.patternSize.height = pattern_size_mm;

  sub_key_ = nh_.subscribe("/key", 30, &PointCloudAligner::KeyCallback, this);

  sub_selected_points_ = nh_.subscribe("/rviz_selected_points",
                                       30, &PointCloudAligner::CallbackClickedPoint,
                                       this);

  std::string data_path = ros::package::getPath("extrinsic_calibrator") + "/data";
  std::vector<std::string> vector_file_names = DataWriteRead::get_file_names(data_path);
  total_sample_count = vector_file_names.size() / 3;
  vector_samples = DataWriteRead::read_samples(total_sample_count, data_path);
  std::cout << "Total sample count:" << vector_samples.size() << std::endl;

  trans_optimized.setIdentity();

  // mode 1 calibration
  // mode 2 test

  if (mode == 1 or mode==2)
  {
    // Use this params to test cam lid calibration:
    Eigen::Quaterniond q_optimized = {vector_q_param[0],
                                      vector_q_param[1],
                                      vector_q_param[2],
                                      vector_q_param[3]};
    trans_optimized.linear() = q_optimized.toRotationMatrix();
    trans_optimized.translation() = Eigen::Vector3d {vector_trans_param[0],
                                                     vector_trans_param[1],
                                                     vector_trans_param[2]};


    timer_pc_publisher_ = nh_.createTimer(ros::Duration(0.1),
                                          &PointCloudAligner::CallbackSamplePublisher,
                                          this);

    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud", 1, this);
    pub_cloud_plane_points_cam_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_plane_points_cam", 1, this);
    pub_cloud_plane_points_lid_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_plane_points_lid", 1, this);
    pub_cloud_frustum_colored = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_colored", 1, this);
    pub_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>("/plane_normals", 1, this);
    pub_image_ = it.advertise("/image", 10);
    pub_undistorted_image_ = it.advertise("/undistorted_image", 10);
  }
  else {
    std::cout << "Use appropriate mode id!" << std::endl;
  }

  std::cout << "MODE:" << mode << std::endl;

}

void
PointCloudAligner::KeyCallback(
    const std_msgs::Int32::ConstPtr &msg) {
  // press 'n' : next pair
  if (msg->data == 110 and sample_id < total_sample_count-1) {
    sample_id++;
    std::cout << "Next sample, id:" << sample_id << std::endl;
  }

  // press 'p' : previous
  if (msg->data == 112) {
    std::cout << "Previous sample, id:" << sample_id << std::endl;
    sample_id--;
  }

  // press 'e' : to extract features:
  if (msg->data == 101) {

    Cloud::Ptr cloud_plane_cam = FeatureExtractor::extract_image_features(
        vector_samples[sample_id],
        calibration_parameters_,
        pub_cloud_plane_points_cam_);
    vector_all_planes_cam.push_back(cloud_plane_cam);

    if (!cloud_plane_cam->points.empty())
    {
      // Not automatic:
      cloud_selected_points->points.clear();
      cloud_selected_points = vector_samples[sample_id].cloud;

      // ############################################################################

      Cloud::Ptr cloud_plane_lid = FeatureExtractor::extract_lidar_features(vector_samples[sample_id],
                                                                            cloud_selected_points,
                                                                            pub_cloud_plane_points_lid_);
      vector_all_planes_lid.push_back(cloud_plane_lid);


      if (!vector_samples[sample_id].cloud_plane_points_lidar->points.empty()) {
        vector_samples[sample_id].is_feature_extracted = true;

        marker_array_.markers.clear();
        ROSHandler::PutArrowMarker(vector_samples[sample_id].normal_vector_cam,
                                   vector_samples[sample_id].middle_point_cam,
                                   marker_array_, marker_id_);
        ROSHandler::PutArrowMarker(vector_samples[sample_id].normal_vector_lid,
                                   vector_samples[sample_id].middle_point_lid,
                                   marker_array_, marker_id_);
        pub_marker_array_.publish(marker_array_);

        std::cout << "Calibration features are extracted." << std::endl;
      }
    } else {
      std::cout << "Image feature extraction is not succeed." << std::endl;
    }



    // Calculate test stuff:
    if (mode==2)
    {
      Test::project_lidar_points_to_image_plane(vector_samples[sample_id],
                                                calibration_parameters_,
                                                trans_optimized,
                                                pub_undistorted_image_,
                                                pub_cloud_frustum_colored,
                                                max_distance_cam_seen);
    }


  }

  // press 'o' : to start optimization
  if (msg->data == 111) {

    std::cout << "Results parent frame (camera), child frame(lidar)." << std::endl;
    // Solve rotation by aligning lidar and camera plane normals
    RotationSolver rotation_solver(vector_samples);
    Eigen::Quaterniond q_optimized = rotation_solver.solve();

    // Rotate the whole lidar plane points:
    Eigen::Affine3d tf_rotator;
    tf_rotator.setIdentity();
    tf_rotator.linear() = q_optimized.toRotationMatrix();
    tf_rotator.translation() = Eigen::Vector3d {0,0,0};
    for (auto& sample:vector_samples)
    {
      if (sample.is_feature_extracted)
      {
        pcl::transformPointCloud(*sample.cloud_plane_points_lidar,
                                 *sample.cloud_plane_points_lidar,
                                 tf_rotator);
      }
    }

    ROSHandler::PublishCloud(pub_cloud_plane_points_lid_, vector_samples[sample_id].cloud_plane_points_lidar,
                             "camera_frame");


    /*
     * Solve translation by minimizing distance between lidar plane points and camera planes
     * after finding rotation
     */

    TranslationSolver translation_solver(vector_samples);
    Eigen::Vector3d translation_optimized = translation_solver.solve();




    /*std::string path_output_folder = ros::package::getPath("extrinsic_calibrator") + "/output_folder";
    for (size_t i=0; i<vector_all_planes_lid.size(); i++)
    {
      std::string name_target = path_output_folder + "/" + std::to_string(i) + "_cam.pcd";
      pcl::io::savePCDFile(name_target,*vector_all_planes_cam.at(i));
      std::string name_source = path_output_folder + "/" + std::to_string(i) + "_lid.pcd";
      pcl::transformPointCloud(*vector_all_planes_lid.at(i), *vector_all_planes_lid.at(i), tf_optimized);
      pcl::io::savePCDFile(name_source,*vector_all_planes_lid.at(i));
    }*/

    /*PointToPlaneSolver point_to_plane_solver(vector_samples);
    point_to_plane_solver.solve();*/

    std::cout << "Optimization is done." << std::endl;

    trans_optimized.linear() = q_optimized.toRotationMatrix();
    trans_optimized.translation() = translation_optimized;
    Eigen::Vector3d ea = q_optimized.toRotationMatrix().eulerAngles(0, 1, 2); 
    std::cout << "yaw: " << ea(2)*57.29577 << " pitch: " << ea(1)*57.29577 << " roll: " << ea(0)*57.29577 << std::endl;
    mode = 2;
  }
}

void
PointCloudAligner::CallbackSamplePublisher(const ros::TimerEvent &) {

  if (mode ==2)
  {
    Cloud::Ptr cloud_roi_transformed(new Cloud);
    pcl::transformPointCloud(*vector_samples[sample_id].cloud, *cloud_roi_transformed, trans_optimized);
    ROSHandler::PublishCloud(pub_cloud_, cloud_roi_transformed, "camera_frame");
    ROSHandler::PublishImage(pub_image_, vector_samples[sample_id].image, "camera_frame");
  } else
  {
    ROSHandler::PublishCloud(pub_cloud_, vector_samples[sample_id].cloud, "camera_frame");
    ROSHandler::PublishImage(pub_image_, vector_samples[sample_id].image, "camera_frame");
  }

}

void
PointCloudAligner::CallbackClickedPoint(
    const sensor_msgs::PointCloud2ConstPtr &msg_cloud_selected_points) {
  std::cout << "Lidar plane points are selected." << std::endl;
  cloud_selected_points->points.clear();
  pcl::fromROSMsg(*msg_cloud_selected_points,
                  *cloud_selected_points);

}

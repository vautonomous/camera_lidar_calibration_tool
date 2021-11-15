#include "data_write_read.h"

std::vector<std::string>
DataWriteRead::get_file_names(const std::string &pcd_path) {
  std::vector<std::string> vector_path;

  for (auto i = boost::filesystem::directory_iterator(pcd_path);
       i != boost::filesystem::directory_iterator(); i++) {
    if (!is_directory(i->path())) {
      std::string single_pcd_path = pcd_path + i->path().
          filename().string();
      vector_path.push_back(single_pcd_path);
    } else
      continue;
  }
  return vector_path;
}

std::vector<Sample>
DataWriteRead::read_samples(const size_t &total_sample_count,
                            const std::string &data_path) {
  std::vector<Sample> samples;
  for (size_t i = 0; i < total_sample_count; i++) {
    std::string path_pcd = data_path + "/" + std::to_string(i) + "_cloudROI.pcd";
    std::string path_image = data_path + "/" + std::to_string(i) + ".jpg";
    cv::Mat image = cv::imread(path_image, cv::IMREAD_COLOR);
    Cloud::Ptr cloud(new Cloud);
    pcl::io::loadPCDFile<Point>(path_pcd, *cloud);

    std::string path_pcd_test = data_path + "/" + std::to_string(i) + "_test_cloud.pcd";
    Cloud::Ptr cloud_test(new Cloud);
    pcl::io::loadPCDFile<Point>(path_pcd_test, *cloud_test);

    Sample sample;
    sample.set_image(image);
    sample.set_cloud(cloud);
    sample.set_test_cloud(cloud_test);
    samples.push_back(sample);
  }
  return samples;
}

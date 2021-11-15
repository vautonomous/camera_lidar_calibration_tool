

#include <iostream>
#include <vector>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <sample.h>

class DataWriteRead
{
 public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;

  static std::vector<std::string>
  get_file_names(const std::string& pcd_path);

  static std::vector<Sample>
  read_samples(const size_t& total_sample_count,
               const std::string& data_path);



};


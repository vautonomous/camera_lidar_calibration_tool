#ifndef SRC_EXTRINSIC_CALIBRATOR_INCLUDE_CALIBRATION_PARAMETERS_H_
#define SRC_EXTRINSIC_CALIBRATOR_INCLUDE_CALIBRATION_PARAMETERS_H_

#include <opencv4/opencv2/highgui.hpp>

class CalibrationParameters
{
public:
  cv::Mat distcoeff;
  cv::Mat cameramat;
  cv::Size2i patternNum;
  cv::Size2i patternSize;
};



#endif  // SRC_EXTRINSIC_CALIBRATOR_INCLUDE_CALIBRATION_PARAMETERS_H_


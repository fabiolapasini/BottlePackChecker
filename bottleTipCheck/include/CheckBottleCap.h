
#pragma once

#include <string.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "utils.h"

// point cloud library
#include <pcl/common/transforms.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

// pcl io
#include <pcl/io/file_io.h>
#include <pcl/io/pcd_io.h>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv_modules.hpp"

namespace QualityChecker {

void loadAndProcessDepthImage(const std::string& depth_name, int start_r,
    int start_c, int w_depth, int h_depth, float threshold_z, cv::Mat& depth,
    float& minD, float& maxD, log4cxx::LoggerPtr logger);

void loadAndProcessRGBImage(const std::string& file_name, float threshold_z,
                         float& minD, float& maxD, cv::Mat& image_rgb,
                         log4cxx::LoggerPtr logger);

void checkLogo(AppState& state);

}  // namespace QualityChecker
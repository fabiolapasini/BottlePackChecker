#include <string.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// point cloud library
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv_modules.hpp"
// #include "opencv2/xfeatures2d/nonfree.hpp"

#include "../include/checkBottleCap.h"

using Pointcloud = pcl::PointCloud<pcl::PointXYZ>;

void loadAndProcessDepthImage(const std::string& pcl_path,
                              const std::string& file_name,
                              const std::string& depth_name, int start_r,
                              int start_c, int w_depth, int h_depth,
                              float threshold_z, cv::Mat& depth,
                              log4cxx::LoggerPtr logger) {
  Pointcloud pc_depth;
  if (pcl::io::loadPCDFile(pcl_path + depth_name + ".pcd", pc_depth) < 0) {
    LOG4CXX_ERROR(logger, "Error loading depth pointcloud file: "
                              << pcl_path + depth_name + ".pcd");
    return;
  }

  float minD = std::numeric_limits<float>::max();
  float maxD = 0.f;

  for (int i = 0; i < h_depth; i++) {
    for (int j = 0; j < w_depth; j++) {
      /* Get the depth value (z-axis) from the point at the correct position,
      considering the row (start_r) and column (start_c) offsets in the depth
      point cloud */
      float val = pc_depth[(i + start_r) * pc_depth.width + (j + start_c)].z;
      if (val < threshold_z && !std::isnan(val)) {
        // Create the image
        depth.at<float>(i, j) = 1.f - val;
        if (val < minD) minD = val;
        if (val > maxD) maxD = val;
      }
    }
  }

  LOG4CXX_DEBUG(logger,
                "Depth image stats: minD = " << minD << ", maxD = " << maxD);

  // constant stretching
  depth -= minD;
  depth /= (maxD - minD);
}

void QualityChecker::checkLogo(AppState& state) {
  cv::Mat depth(state.config.roi.h_depth, state.config.roi.w_depth, CV_32FC1,
                cv::Scalar(0));

  // Extract and crop the depth image for the kernel
  loadAndProcessDepthImage(
      state.config.foldersInfo.PCLFiles, state.config.filesInfo.InputName,
      state.config.filesInfo.OutputName, state.config.roi.start_r,
      state.config.roi.start_c, state.config.roi.w_depth,
      state.config.roi.h_depth, state.config.thresholds.ThresholdZ, depth,
      state.logger);

  return;
}

#pragma once

#include <string.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "utils.h"
#include "IDetector.h"

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

typedef pcl::PointCloud<pcl::PointXYZRGBA> Pointcloud;

namespace QualityChecker 
{

//double getDepthFromPointCloud(const cv::Point2d& pt,
//                              const rs2::points& pointcloud, double minD,
//                              double oldmin);

cv::Point2d computeOffsetInMeters(const cv::Point2d& pt1, double z1,
                                const cv::Point2d& pt2, double z2, double fx,
                                double fy, double u0, double v0, double minD,
                                double oldmin);

void convolveAndFindAlignment(const cv::Mat& depth_scale,
                                const cv::Mat& depth_image_scale,
                              cv::Mat& conv, float angle, double& x_c,
                                double& y_c, float& best_angle);

void translatePointcloudAlongZ(const Pointcloud::Ptr& input_cloud,
                               Pointcloud::Ptr& output_cloud,
                                            float old_min_depth,
                                            float new_min_depth);

void loadAndProcessDepthImage(const std::string& depth_name, int start_r,
    int start_c, int w_depth, int h_depth, float threshold_z, cv::Mat& depth,
    float& minD, float& maxD, log4cxx::LoggerPtr logger);

void loadAndProcessRGBImage(Pointcloud::Ptr& pointcloud,
                                            const std::string& file_name,
                                            float threshold_z, float& minD,
                                            float& maxD, cv::Mat& image_rgb,
                                            log4cxx::LoggerPtr logger);

void checkLogo(AppState& state, IDetector& detector);

}  // namespace QualityChecker
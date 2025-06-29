

#include "../include/checkBottleCap.h"

#include "../include/paths.h"

typedef pcl::PointCloud<pcl::PointXYZRGBA> Pointcloud;

void QualityChecker::loadAndProcessDepthImage(const std::string& depth_name,
                                              int start_r, int start_c,
                                              int w_depth, int h_depth,
                                              float threshold_z, cv::Mat& depth,
                                              float& minD, float& maxD,
                                              log4cxx::LoggerPtr logger) {

  Pointcloud::Ptr pc_depth(new Pointcloud);
  std::filesystem::path file_path = Paths::pcl_path / (depth_name + ".pcd");
  if (pcl::io::loadPCDFile(file_path.string(), *pc_depth) < 0) {
    LOG4CXX_ERROR(
        logger, "Error loading depth pointcloud file: " << file_path.string());
    return;
  }

  LOG4CXX_DEBUG(logger, "Trying to load file: " << file_path.string());

  if (!pc_depth->isOrganized()) {
    LOG4CXX_ERROR(logger,
                  "Pointcloud is not organized. Aborting depth extraction.");
    return;
  }

  for (int i = 0; i < h_depth; ++i) {
    for (int j = 0; j < w_depth; ++j) {
      int row = i + start_r;
      int col = j + start_c;

      if (row >= pc_depth->height || col >= pc_depth->width) {
        continue;  // Skip out-of-bounds
      }

      float val = pc_depth->at(col, row).z;

      if (val < threshold_z && std::isfinite(val)) {
        /* Take the inverse to visualize closer values
        as lighter and farther ones as darker */
        depth.at<float>(i, j) = 1.f - val;

        // Update min and max value
        if (val < minD) minD = val;
        if (val > maxD) maxD = val;
      }
    }
  }

  LOG4CXX_DEBUG(logger,
                "Depth image stats: min depth img = " << minD << ", max depth img = " << maxD);

  // constant stretching
  if (maxD > minD) {
    depth -= minD;
    depth /= (maxD - minD);
  } else {
    LOG4CXX_WARN(logger, "No valid depth values for normalization.");
  }
}

void QualityChecker::loadAndProcessRGBImage(const std::string& file_name,
                                            float threshold_z, float& minD,
                                            float& maxD, cv::Mat& image_rgb,
                                            log4cxx::LoggerPtr logger) {
  Pointcloud::Ptr pointcloud(new Pointcloud);
  std::filesystem::path file_path = Paths::pcl_path / (file_name + ".pcd");

  if (pcl::io::loadPCDFile(file_path.string(), *pointcloud) < 0) {
    LOG4CXX_ERROR(logger,
                  "Error loading RGB pointcloud file: " << file_path.string());
    return;
  }

  LOG4CXX_DEBUG(logger, "Successfully loaded: " << file_path.string());

  if (!pointcloud->isOrganized()) {
    LOG4CXX_ERROR(logger, "Pointcloud is not organized. Cannot extract image.");
    return;
  }

  // Resize or reallocate the image if needed
  image_rgb.create(pointcloud->height, pointcloud->width, CV_8UC3);
  image_rgb.setTo(cv::Scalar(0, 0, 0));  // Clear to black

  for (int i = 0; i < pointcloud->height; ++i) {
    for (int j = 0; j < pointcloud->width; ++j) {
      auto& pt = pointcloud->at(j, i);
      image_rgb.at<cv::Vec3b>(i, j)[0] = pt.b;
      image_rgb.at<cv::Vec3b>(i, j)[1] = pt.g;
      image_rgb.at<cv::Vec3b>(i, j)[2] = pt.r;

      if (pt.z < threshold_z && std::isfinite(pt.z)) {
        if (pt.z < minD) minD = pt.z;
        if (pt.z > maxD) maxD = pt.z;
      }
    }
  }

  LOG4CXX_DEBUG(
      logger, "RGB image depth stats: min rgb img = " << minD << ", max rgb img = " << maxD);
}

void QualityChecker::checkLogo(AppState& state) {
  // Extract and crop the depth image later used as kernel of a convolution
  cv::Mat depth(state.config.roi.h_depth, state.config.roi.w_depth, CV_32FC1,
                cv::Scalar(0));
  float min_depth_img = std::numeric_limits<float>::max();
  float max_depth_img = 0.0f;
  loadAndProcessDepthImage(state.config.filesInfo.DepthName,
                           state.config.roi.start_r, state.config.roi.start_c,
                           state.config.roi.w_depth, state.config.roi.h_depth,
                           state.config.thresholds.ThresholdZ, depth,
                           min_depth_img, max_depth_img, state.logger);

  cv::Mat image_rgb;
  float min_rgb_img = std::numeric_limits<float>::max();
  float max_rgb_img = 0.0f;
  loadAndProcessRGBImage(state.config.filesInfo.InputName,
                         state.config.thresholds.ThresholdZ, min_rgb_img,
                         max_rgb_img, image_rgb, state.logger);

  /*  Resize the grayscale RGB image so that the apparent object scale matches
   the depth kernel. This is done by scaling with the ratio between the
   minimum depth value in the RGB image and in the kernel region. */
  cv::Mat image, image_gray;
  cv::cvtColor(image_rgb, image_gray, cv::COLOR_RGB2GRAY, 0);
  if (min_depth_img > 0.0f) {
    float scale = min_rgb_img / min_depth_img;
    cv::resize(image_gray, image,
               cv::Size(image_gray.cols * scale, image_gray.rows * scale));
  } else {
    LOG4CXX_WARN(state.logger, "Invalid min_depth_img value for resizing.");
    image = image_gray.clone();
  }

  return;
}
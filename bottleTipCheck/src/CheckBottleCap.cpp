
#include "../include/checkBottleCap.h"
#include "../include/paths.h"


 double QualityChecker::getDepthFromPointCloud(const cv::Point2d& pt,
                                              const Pointcloud::Ptr & pointcloud,
                                              double minD, double oldmin) {
   int row = static_cast<int>(pt.y / (minD / oldmin));
   int col = static_cast<int>(pt.x / (minD / oldmin));

   int idx = col + row * pointcloud->width;
   float z = pointcloud->at(idx).z;

   if (!std::isnan(z)) {
     return z;
   }

   // if it is NaN, I look for valid pixel in the neigborhood
   int min_dist_row = INT_MAX;
   int min_dist_col = INT_MAX;

   int i = 1;
   while (row + i < pointcloud->height &&
          std::isnan(pointcloud->at((row + i) * pointcloud->width + col).z))
     ++i;
   if (row + i < pointcloud->height) min_dist_row = i;

   i = 1;
   while (row - i >= 0 &&
          std::isnan(pointcloud->at((row - i) * pointcloud->width + col).z))
     ++i;
   if (row - i >= 0 && i < min_dist_row) min_dist_row = -i;

   int j = 1;
   while (col + j < pointcloud->width &&
          std::isnan(pointcloud->at(row * pointcloud->width + col + j).z))
     ++j;
   if (col + j < pointcloud->width) min_dist_col = j;

   j = 1;
   while (col - j >= 0 &&
          std::isnan(pointcloud->at(row * pointcloud->width + col - j).z))
     ++j;
   if (col - j >= 0 && j < abs(min_dist_col)) min_dist_col = -j;

   if (abs(min_dist_row) < abs(min_dist_col)) {
     return pointcloud->at((row + min_dist_row) * pointcloud->width + col).z;
   } else {
     return pointcloud->at(row * pointcloud->width + col + min_dist_col).z;
   }
 }

cv::Point2d QualityChecker::computeOffsetInMeters(
    const cv::Point2d& pt1, double z1, const cv::Point2d& pt2, double z2,
    double fx, double fy, double u0, double v0, double minD, double oldmin) {
  double x1_m = (pt1.x / (minD / oldmin) - u0 + 0.5) * z1 / fx;
  double y1_m = (pt1.y / (minD / oldmin) - v0 + 0.5) * z1 / fy;

  double x2_m = (pt2.x / (minD / oldmin) - u0 + 0.5) * z2 / fx;
  double y2_m = (pt2.y / (minD / oldmin) - v0 + 0.5) * z2 / fy;

  return cv::Point2d(x1_m - x2_m, y1_m - y2_m);
}

/* Performs convolution between rotated depth template and depth image
 Returns the center coordinates (x_c, y_c) and the best matching rotation
 angle */
void QualityChecker::convolveAndFindAlignment(
    const cv::Mat& depth_scale, const cv::Mat& depth_current_image_scale,
    cv::Mat& conv, float angle, double& x_c, double& y_c, float& best_angle) {
  float minVal = std::numeric_limits<float>::max();

  // Rotate the kernel around its center
  cv::Mat rotated_depth_scale;
  cv::Point2f center((depth_scale.cols - 1) / 2.0f,
                     (depth_scale.rows - 1) / 2.0f);
  cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, -angle, 1.0);
  cv::warpAffine(depth_scale, rotated_depth_scale, rotation_matrix,
                 depth_scale.size());

  // Slide kernel over image to compute convolution
  for (int i = 0; i < depth_current_image_scale.rows - depth_scale.rows; ++i) {
    for (int j = 0; j < depth_current_image_scale.cols - depth_scale.cols;
         ++j) {
      float val = 0.0f;
      for (int k = 0; k < depth_scale.rows; ++k) {
        for (int l = 0; l < depth_scale.cols; ++l) {
          float pixelIm = depth_current_image_scale.at<float>(i + k, j + l);
          float pixelElem = depth_scale.at<float>(k, l);
          val += (pixelElem - pixelIm) * (pixelElem - pixelIm);
        }
      }

      conv.at<float>(i + depth_scale.rows / 2, j + depth_scale.cols / 2) = val;

      if (val < minVal) {
        minVal = val;
        y_c = i + depth_scale.rows / 2.0;
        x_c = j + depth_scale.cols / 2.0;
      }
    }
  }

  // Refine rotation to find best match angle near initial
  minVal = std::numeric_limits<float>::max();
  for (float a = angle - 3.f; a <= angle + 3.f; a += 0.5f) {
    cv::Mat rotated_depth_current_image_scale;
    cv::Point2f center_im(x_c, y_c);
    cv::Mat rot_im = cv::getRotationMatrix2D(center_im, -a, 1.0);
    cv::warpAffine(depth_current_image_scale, rotated_depth_current_image_scale,
                   rot_im, depth_current_image_scale.size());

    float val = 0.0f;
    for (int k = 0; k < rotated_depth_scale.rows; ++k) {
      for (int l = 0; l < rotated_depth_scale.cols; ++l) {
        float pixelIm = rotated_depth_current_image_scale.at<float>(
            y_c - rotated_depth_scale.rows / 2.f + k,
            x_c - rotated_depth_scale.cols / 2.f + l);
        float pixelElem = rotated_depth_scale.at<float>(k, l);
        val += (pixelElem - pixelIm) * (pixelElem - pixelIm);
      }
    }

    if (val < minVal) {
      minVal = val;
      best_angle = a;
    }
  }
}

void QualityChecker::translatePointcloudAlongZ(
    const Pointcloud::Ptr& input_cloud, Pointcloud::Ptr& output_cloud,
    float old_min_depth, float new_min_depth) {
  // Compute the difference between old and new minimum depth
  float diff = new_min_depth - old_min_depth;

  // Create 4x4 homogeneous transformation matrix (identity)
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // Set translation along the Z-axis (depth)
  transform(2, 3) = -diff;

  // Apply the transformation to the point cloud
  pcl::transformPointCloud(*input_cloud, *output_cloud, transform);
}

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

  LOG4CXX_DEBUG(logger, "Depth image stats: min depth img = "
                            << minD << ", max depth img = " << maxD);

  // constant stretching
  if (maxD > minD) {
    depth -= minD;
    depth /= (maxD - minD);
  } else {
    LOG4CXX_WARN(logger, "No valid depth values for normalization.");
  }
}

void QualityChecker::loadAndProcessRGBImage(Pointcloud::Ptr& pointcloud,
                                            const std::string& file_name,
                                            float threshold_z, float& minD,
                                            float& maxD,
                                            cv::Mat& rgb_current_img,
                                            log4cxx::LoggerPtr logger) {
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
  rgb_current_img.create(pointcloud->height, pointcloud->width, CV_8UC3);
  rgb_current_img.setTo(cv::Scalar(0, 0, 0));  // Clear to black

  for (int i = 0; i < pointcloud->height; ++i) {
    for (int j = 0; j < pointcloud->width; ++j) {
      auto& pt = pointcloud->at(j, i);
      rgb_current_img.at<cv::Vec3b>(i, j)[0] = pt.b;
      rgb_current_img.at<cv::Vec3b>(i, j)[1] = pt.g;
      rgb_current_img.at<cv::Vec3b>(i, j)[2] = pt.r;

      if (pt.z < threshold_z && std::isfinite(pt.z)) {
        if (pt.z < minD) minD = pt.z;
        if (pt.z > maxD) maxD = pt.z;
      }
    }
  }

  LOG4CXX_DEBUG(logger, "RGB image depth stats: min rgb img = "
                            << minD << ", max rgb img = " << maxD);
}

/////////////////////////////////////////////////////////////////////////////////////////////

void QualityChecker::checkLogo(AppState& state, IDetector& detector) {
  // Extract and crop the depth image later used as kernel of a convolution
  cv::Mat depth_kernel_img(state.config.roi.h_depth, state.config.roi.w_depth,
                           CV_32FC1, cv::Scalar(0));
  float min_depth_img = std::numeric_limits<float>::max();
  float max_depth_img = 0.0f;

  // TODO(Fabiola): it is a good thing to create a loop: load an image and evaluate it
  // then load automaticallya new image and do the same. Thre is no more the input image but a 
  // an input folder to load images from. 
  loadAndProcessDepthImage(state.config.filesInfo.DepthName,
                           state.config.roi.start_r, state.config.roi.start_c,
                           state.config.roi.w_depth, state.config.roi.h_depth,
                           state.config.thresholds.ThresholdZ, depth_kernel_img,
                           min_depth_img, max_depth_img, state.logger);

  // Load the Logo Image
  std::filesystem::path logo_path =
      Paths::logo_path / (state.config.filesInfo.LogoName + ".png");
  cv::Mat logo_reference_image =
      cv::imread(logo_path.string(), cv::IMREAD_GRAYSCALE);

  // Get the rgb image that I will analize
  cv::Mat rgb_current_img;
  float max_rgb_img = 0.0f;
  Pointcloud::Ptr pointcloud(new Pointcloud);
  float min_rgb_img = std::numeric_limits<float>::max();
  loadAndProcessRGBImage(pointcloud, state.config.filesInfo.InputName,
                         state.config.thresholds.ThresholdZ, min_rgb_img,
                         max_rgb_img, rgb_current_img, state.logger);

  /* Resize the grayscale RGB image so that the apparent object scale matches
   the depth kernel. This is done by scaling with the ratio between the
   minimum depth value in the RGB image and in the kernel region. */
  cv::Mat grey_current_scaled_img, grey_current_img;
  cv::cvtColor(rgb_current_img, grey_current_img, cv::COLOR_RGB2GRAY, 0);
  if (min_depth_img > 0.0f) {
    float scale = min_rgb_img / min_depth_img;
    cv::resize(
        grey_current_img, grey_current_scaled_img,
        cv::Size(grey_current_img.cols * scale, grey_current_img.rows * scale));
  } else {
    LOG4CXX_WARN(state.logger, "Invalid min_depth_img value for resizing.");
    grey_current_scaled_img = grey_current_img.clone();
  }

  // Traslate point cloud
  Pointcloud::Ptr aligned_cloud(new Pointcloud);
  translatePointcloudAlongZ(pointcloud, aligned_cloud, min_rgb_img,
                            min_depth_img);

  // Get the depth image that I will analize
  cv::Mat depth_current_image(pointcloud->height, pointcloud->width, CV_32FC1,
                              cv::Scalar(0));
  for (int i = 0; i < aligned_cloud->height; i++) {
    for (int j = 0; j < aligned_cloud->width; j++) {
      float val = aligned_cloud->at(j, i).z;
      if (val < state.config.thresholds.ThresholdZ && !isnan(val)) {
        depth_current_image.at<float>(i, j) = 1.f - val;
      }
    }
  }

  // Resize and Scale
  cv::resize(
      depth_current_image, depth_current_image,
      cv::Size(depth_current_image.cols * (min_rgb_img / min_depth_img),
               depth_current_image.rows * (min_rgb_img / min_depth_img)));
  depth_current_image -= min_rgb_img;
  depth_current_image /= (max_rgb_img - min_rgb_img);

  // Scales the depth images to reduce computation and shows the result
  cv::Mat depth_kernel_img_scaled, depth_current_image_scale;
  cv::resize(depth_kernel_img, depth_kernel_img_scaled,
             cv::Size(depth_kernel_img.cols / state.config.thresholds.Scale,
                      depth_kernel_img.rows / state.config.thresholds.Scale));
  cv::resize(
      depth_current_image, depth_current_image_scale,
      cv::Size(depth_current_image.cols / state.config.thresholds.Scale,
               depth_current_image.rows / state.config.thresholds.Scale));

  // Show scaled images for debugging
  /* showImagesWithWindows({"Depth Scale", "Depth Image Scale"},
                        {depth_kernel_img_scaled, depth_current_image_scale});
   */

  double x_center = 0.0, y_center = 0.0;
  float best_angle = 0.0f;
  // Initial convolution map and minimum value tracking
  cv::Mat convolution_image(depth_current_image_scale.rows,
                            depth_current_image_scale.cols, CV_32FC1,
                            cv::Scalar(0));
  convolveAndFindAlignment(
      depth_kernel_img_scaled, depth_current_image_scale, convolution_image,
      state.config.thresholds.RotationKernel, x_center, y_center, best_angle);

  /* showImagesWithWindows({"Convolution"}, {convolution_image}); */
  LOG4CXX_INFO(state.logger, "best angle: " << best_angle);

  // Bach to original scale
  x_center *= state.config.thresholds.Scale;
  y_center *= state.config.thresholds.Scale;
  LOG4CXX_INFO(state.logger,
               "top_center: x = " << x_center << " y = " << y_center);

  // Detect Logo
  detector.detectLogo(logo_reference_image, grey_current_scaled_img);
  double angle =
      detector.findLogoRotation(logo_reference_image, grey_current_scaled_img);

  double x_top_center =
      x_center - (depth_kernel_img.rows / 2) * sin(best_angle / 180.f);
  double y_top_center =
      y_center - (depth_kernel_img.rows / 2) * cos(best_angle / 180.f);

  // Draw a rectangle around the package
  cv::rectangle(grey_current_scaled_img,
                cv::Rect(cv::Point(x_center - depth_kernel_img.cols / 2,
                                   y_center - depth_kernel_img.rows / 2),
                         cv::Point(x_center + depth_kernel_img.cols / 2,
                                   y_center + depth_kernel_img.rows / 2)),
                cv::Scalar(255));

  // Calcola profondità punto centrale pacco
  cv::Point2d center_pack(x_center, y_center);
  double z_center_pack = getDepthFromPointCloud(center_pack, pointcloud,
                                              min_rgb_img,
                                        min_depth_img);
  LOG4CXX_INFO(state.logger, "center depth: " << z_center_pack);

  // central point of the original logo (logo image coordinates) as vector 3×1 ready for homography
  cv::Mat center_logo =
      (cv::Mat_<double>(3, 1) << logo_reference_image.cols / 2,
       logo_reference_image.rows / 2, 1.0);
  // apply the homography found between logo and scene
  cv::Mat center_logo_img = detector.getHomographyMatrix() * center_logo;
  // normalize because H works in homogeneous coordinates
  center_logo_img /= center_logo_img.at<double>(2, 0);
  cv::Point2d center_logo_pt(center_logo_img.at<double>(0, 0),
                          center_logo_img.at<double>(1, 0));  

  // Calcola profondita centro logo
  double z_center_logo = getDepthFromPointCloud(center_logo_pt, pointcloud,
                                                 min_rgb_img,
                                      min_depth_img);
  LOG4CXX_INFO(state.logger, "logo center depth: " << z_center_logo);

 // Calcola profondità tappo centrale
 double z_top_cap = z_center_pack + state.config.camera.dz;
 LOG4CXX_INFO(state.logger, "central tip depth: " << z_top_cap);

 // Calcola offset
 cv::Point2d center_cap(x_top_center, y_top_center);
 cv::Point2d offset = computeOffsetInMeters(
     center_logo_pt, z_center_logo, center_cap, z_top_cap,
     state.config.camera.fx,
     state.config.camera.fy, state.config.camera.u0, state.config.camera.v0,
     min_rgb_img, min_depth_img);

 LOG4CXX_INFO(state.logger,
              "central cap - logo x-offset (meters): " << offset.x);
 LOG4CXX_INFO(state.logger,
              "central cap - logo y-offset (meters): " << offset.y);

  return;
}
#include "../include/utils.h"

#include <log4cxx/basicconfigurator.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/patternlayout.h>

using namespace log4cxx;
using namespace log4cxx::helpers;

void showImagesWithWindows(const std::vector<std::string>& windowNames,
                           const std::vector<cv::Mat>& images,
                           int waitKeyDelay) {
  if (windowNames.size() != images.size()) return;

  for (size_t i = 0; i < images.size(); ++i) {
    cv::imshow(windowNames[i], images[i]);
  }

  cv::waitKey(waitKeyDelay);

  for (const auto& name : windowNames) {
    cv::destroyWindow(name);
  }
}

// === JSON SERIALIZATION  ===

// FilesInfo
void to_json(json& j, const FilesInfo& p) {
  j = json{{"InputName", p.InputName},
           {"DepthName", p.DepthName},
           {"OutputName", p.OutputName},
           {"LogoName", p.LogoName}};
}

void from_json(const json& j, FilesInfo& p) {
  j.at("InputName").get_to(p.InputName);
  j.at("DepthName").get_to(p.DepthName);
  j.at("OutputName").get_to(p.OutputName);
  j.at("LogoName").get_to(p.LogoName);
}

// Thresholds
void to_json(json& j, const Thresholds& t) {
  j = json{{"ThresholdZ", t.ThresholdZ},
           {"ReprojectionThreshold", t.ReprojectionThreshold},
           {"Scale", t.Scale},
           {"RotationKernel", t.RotationKernel}};
}

void from_json(const json& j, Thresholds& t) {
  j.at("ThresholdZ").get_to(t.ThresholdZ);
  j.at("ReprojectionThreshold").get_to(t.ReprojectionThreshold);
  j.at("Scale").get_to(t.Scale);
  j.at("RotationKernel").get_to(t.RotationKernel);
}

// CameraParameters
void to_json(json& j, const CameraParameters& c) {
  j = json{
      {"FX", c.fx}, {"FY", c.fy}, {"U0", c.u0}, {"V0", c.v0}, {"DZ", c.dz}};
}

void from_json(const json& j, CameraParameters& c) {
  j.at("FX").get_to(c.fx);
  j.at("FY").get_to(c.fy);
  j.at("U0").get_to(c.u0);
  j.at("V0").get_to(c.v0);
  j.at("DZ").get_to(c.dz);
}

// ROI
void to_json(json& j, const ROI& r) {
  j = json{{"StartRow", r.start_r},
           {"StartCol", r.start_c},
           {"WidthDepth", r.w_depth},
           {"HeightDepth", r.h_depth}};
}

void from_json(const json& j, ROI& r) {
  j.at("StartRow").get_to(r.start_r);
  j.at("StartCol").get_to(r.start_c);
  j.at("WidthDepth").get_to(r.w_depth);
  j.at("HeightDepth").get_to(r.h_depth);
}

// AffineSettings
void to_json(json& j, const AffineSettings& a) {
  j = json{{"UseAffine", a.useAffine}};
}

void from_json(const json& j, AffineSettings& a) {
  j.at("UseAffine").get_to(a.useAffine);
}

// Configuration
void to_json(json& j, const Configuration& c) {
  j = json{{"FilesInfo", c.filesInfo},
           {"Thresholds", c.thresholds},
           {"Camera", c.camera},
           {"RegionOfInterest", c.roi},
           {"AffineSettings", c.affine}};
}

void from_json(const json& j, Configuration& c) {
  j.at("FilesInfo").get_to(c.filesInfo);
  j.at("Thresholds").get_to(c.thresholds);
  j.at("Camera").get_to(c.camera);
  j.at("RegionOfInterest").get_to(c.roi);
  j.at("AffineSettings").get_to(c.affine);
}

// === LOGGER IMPLEMENTATION ===

log4cxx::LoggerPtr initLogger(const std::string& loggerName) {
  LoggerPtr logger(Logger::getLogger(loggerName));
  LayoutPtr layout(new PatternLayout("%d [%t] %-5p %c - %m%n"));
  AppenderPtr consoleAppender(
      new ConsoleAppender(layout, ConsoleAppender::getSystemOut()));

  BasicConfigurator::resetConfiguration();
  Logger::getRootLogger()->addAppender(consoleAppender);

  return logger;
}

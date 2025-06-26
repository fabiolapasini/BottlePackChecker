#pragma once

#include <log4cxx/logger.h>

#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

// === DATA STRUCTURES ===

struct FoldersInfo {
  std::string InputFiles;
  std::string LogoFiles;
  std::string OutputFiles;
  std::string PCLFiles;
  std::string DepthImageName;
};

struct FilesInfo {
  std::string InputName;
  std::string OutputName;
};

struct Thresholds {
  double ThresholdZ;
  double ReprojectionThreshold;
  double Scale;
  double RotationKernel;
};

struct CameraParameters {
  double fx;
  double fy;
  double u0;
  double v0;
  double dz;
};

struct ROI {
  int start_r;
  int start_c;
  int w_depth;
  int h_depth;
};

struct AffineSettings {
  bool useAffine;
};

struct Configuration {
  FoldersInfo foldersInfo;
  FilesInfo filesInfo;
  Thresholds thresholds;
  CameraParameters camera;
  ROI roi;
  AffineSettings affine;
};

// === JSON SERIALIZATION DECLARATIONS ===

void to_json(json& j, const FoldersInfo& p);
void from_json(const json& j, FoldersInfo& p);

void to_json(json& j, const FilesInfo& p);
void from_json(const json& j, FilesInfo& p);

void to_json(json& j, const Thresholds& t);
void from_json(const json& j, Thresholds& t);

void to_json(json& j, const CameraParameters& c);
void from_json(const json& j, CameraParameters& c);

void to_json(json& j, const ROI& r);
void from_json(const json& j, ROI& r);

void to_json(json& j, const AffineSettings& a);
void from_json(const json& j, AffineSettings& a);

void to_json(json& j, const Configuration& c);
void from_json(const json& j, Configuration& c);


// === LOGGER ===

log4cxx::LoggerPtr initLogger(const std::string& loggerName = "MyAppLogger");

struct AppState {
  log4cxx::LoggerPtr logger;
  Configuration config;
};

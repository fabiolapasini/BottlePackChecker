#include <fstream>
#include <iostream>
#include <string>

#include "include/checkBottleCap.h"
#include "include/paths.h"
#include "include/utils.h"

using namespace log4cxx;
using namespace log4cxx::helpers;

int main(int argc, char** argv) {
	AppState state;

	// Configure Logger
	state.logger = initLogger("MyTeststate.logger");

	// Read configs
	std::filesystem::path config_json_file = Paths::assets_path / "config.json";
	if (!std::filesystem::exists(config_json_file)) {
		LOG4CXX_ERROR(state.logger,
			"config.json does not exist: " << config_json_file.string());
		return 1;
	}
	std::ifstream file(config_json_file);
	if (!file.is_open()) {
		LOG4CXX_ERROR(state.logger, "Impossible to open config.json");
		return 1;
	}

	json j;
	file >> j;
	state.config = Configuration();
	try {
		state.config = j.get<Configuration>();
	}
	catch (const json::exception& e) {
		LOG4CXX_ERROR(state.logger, "Error during JSON parsing: " << e.what());
		return 1;
	}

	LOG4CXX_INFO(state.logger,
		"FoldersInfo:\n"
		"  InputFiles: "<< Paths::image_path<< "\n"
		"  LogoFiles: "<< Paths::logo_path<< "\n"
		"  OutputFiles: "<< Paths::output_path<< "\n"
		"  PCLFiles: "<< Paths::pcl_path<< "\n"
		"  DepthImageName: " << Paths::depth_path << "\n"
		"FilesInfo:\n"
		"  InputName: " << state.config.filesInfo.InputName << "\n"
		"  OutputName: " << state.config.filesInfo.OutputName << "\n"
		"Thresholds:\n"
		"  ThresholdZ: " << state.config.thresholds.ThresholdZ << "\n"
		"  ReprojectionThreshold: " << state.config.thresholds.ReprojectionThreshold << "\n"
		"  Scale: " << state.config.thresholds.Scale << "\n"
		"  RotationKernel: " << state.config.thresholds.RotationKernel << "\n"
		"CameraParameters:\n"
		"  FX: " << state.config.camera.fx << "\n"
		"  FY: " << state.config.camera.fy << "\n"
		"  U0: " << state.config.camera.u0 << "\n"
		"  V0: " << state.config.camera.v0 << "\n"
		"  DZ: " << state.config.camera.dz << "\n"
		"RegionOfInterest:\n"
		"  StartRow: " << state.config.roi.start_r << "\n"
		"  StartCol: " << state.config.roi.start_c << "\n"
		"  WidthDepth: " << state.config.roi.w_depth << "\n"
		"  HeightDepth: " << state.config.roi.h_depth << "\n"
		"AffineSettings:\n"
		"  UseAffine: " << (state.config.affine.useAffine ? "true" : "false")
	);

	try {
		QualityChecker::checkLogo(state);
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
	}
	catch (...) {
		std::cerr << "Unknown Error" << std::endl;
	}

	return 0;
}
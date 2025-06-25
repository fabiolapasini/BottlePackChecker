#include <fstream>
#include <iostream>
#include <string>

#include "include/paths.h"
#include "include/utils.h"
#include "include/checkBottleCap.h"

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
  } catch (const json::exception& e) {
    LOG4CXX_ERROR(state.logger, "Error during JSON parsing: " << e.what());
    return 1;
  }

  LOG4CXX_INFO(state.logger,
               "FoldersInfo - InputFiles: "
                   << state.config.foldersInfo.InputFiles
          << " IntermediateFiles: " << state.config.foldersInfo.LogosFiles
          << " OutputFiles: " << state.config.foldersInfo.OutputFiles);

  if (state.config.filesInfo.InputName.has_value()) {
    LOG4CXX_INFO(state.logger, "FilesInfo - InputName: "
                     << *state.config.filesInfo.InputName
                                           << " OutputName: "
                                           << state.config.filesInfo.OutputName);
  } else {
    LOG4CXX_INFO(state.logger, "FilesInfo - InputName: <none>"
                                   << " OutputName: "
                                   << state.config.filesInfo.OutputName);
  }

  try {
    QualityChecker::checkLogo(state);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Unknown Error" << std::endl;
  }


  return 0;
}
#include <fstream>
#include <iostream>
#include <string>

#include "include/paths.h"
#include "include/utils.h"
// #include "include/bottleTipCheck.h"

using json = nlohmann::json;
using namespace log4cxx;
using namespace log4cxx::helpers;

int main(int argc, char** argv) {
  // Configure Logger
  LoggerPtr logger(Logger::getLogger("MyTestLogger"));
  LayoutPtr layout(new PatternLayout("%d [%t] %-5p %c - %m%n"));
  AppenderPtr consoleAppender(
      new ConsoleAppender(layout, ConsoleAppender::getSystemOut()));
  BasicConfigurator::resetConfiguration();
  Logger::getRootLogger()->addAppender(consoleAppender);

  // Read configs
  std::filesystem::path config_json_file = assets_path / "config.json";
  if (!std::filesystem::exists(config_json_file)) {
    LOG4CXX_ERROR(logger,
                  "config.json does not exist: " << config_json_file.string());
    return 1;
  }
  std::ifstream file(config_json_file);
  if (!file.is_open()) {
    LOG4CXX_ERROR(logger, "Impossible to open config.json");
    return 1;
  }

  json j;
  file >> j;
  Configuration config;
  try {
    config = j.get<Configuration>();
  } catch (const json::exception& e) {
    LOG4CXX_ERROR(logger, "Error during JSON parsing: " << e.what());
    return 1;
  }

  LOG4CXX_INFO(logger,
               "FoldersInfo - InputFiles: "
                   << config.foldersInfo.InputFiles
                   << " IntermediateFiles: " << config.foldersInfo.LogosFiles
                   << " OutputFiles: " << config.foldersInfo.OutputFiles);

  if (config.filesInfo.InputName.has_value()) {
    LOG4CXX_INFO(logger, "FilesInfo - InputName: "
                             << *config.filesInfo.InputName
                             << " OutputName: " << config.filesInfo.OutputName);
  } else {
    LOG4CXX_INFO(logger, "FilesInfo - InputName: <none>"
                             << " OutputName: " << config.filesInfo.OutputName);
  }

  // checkLogo();
  return 0;
}
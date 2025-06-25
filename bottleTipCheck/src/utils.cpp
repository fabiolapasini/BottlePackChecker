#include <filesystem>
#include <fstream>
#include <sstream>

#include "../include/utils.h"


// LOGGER

using namespace log4cxx;
using namespace log4cxx::helpers;

log4cxx::LoggerPtr initLogger(const std::string &loggerName) 
{
  log4cxx::LoggerPtr logger(Logger::getLogger(loggerName));
  log4cxx::LayoutPtr layout(new PatternLayout("%d [%t] %-5p %c - %m%n"));

  AppenderPtr consoleAppender(
      new ConsoleAppender(layout, ConsoleAppender::getSystemOut()));

  BasicConfigurator::resetConfiguration();
  Logger::getRootLogger()->addAppender(consoleAppender);

  return logger;
}


// SERIALIZATION

using json = nlohmann::json;

// conversion functions for FilesInfo
void to_json(json &j, const FilesInfo &mpi) {
  j = json{{"InputName",
            mpi.InputName.has_value() ? json(*mpi.InputName) : json(nullptr)},
           {"OutputName", mpi.OutputName}};
}

void from_json(const json &j, FilesInfo &mpi) {
  if (j.contains("InputName") && !j["InputName"].is_null())
    mpi.InputName = j["InputName"].get<std::string>();
  else
    mpi.InputName = std::nullopt;

  j.at("OutputName").get_to(mpi.OutputName);
}

// conversion functions for FoldersInfo
void to_json(json &j, const FoldersInfo &fi) {
  j = json{{"InputFiles", fi.InputFiles},
           {"LogoFiles", fi.LogosFiles},
           {"OutputFiles", fi.OutputFiles}};
}

void from_json(const json &j, FoldersInfo &fi) {
  j.at("InputFiles").get_to(fi.InputFiles);
  j.at("LogoFiles").get_to(fi.LogosFiles);
  j.at("OutputFiles").get_to(fi.OutputFiles);
}

// conversion functions for Configuration
void to_json(json &j, const Configuration &config) {
  j = json{{"FilesInfo", config.filesInfo},
           {"FoldersInfo", config.foldersInfo}};
}

void from_json(const json &j, Configuration &config) {
  j.at("FilesInfo").get_to(config.filesInfo);
  j.at("FoldersInfo").get_to(config.foldersInfo);
}

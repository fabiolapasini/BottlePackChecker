#pragma once

#include <log4cxx/asyncappender.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/helpers/pool.h>
#include <log4cxx/logmanager.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/propertyconfigurator.h>

#include <nlohmann/json.hpp>
#include <string>
#include <optional> 

using json = nlohmann::json;


struct FoldersInfo 
{
  std::string InputFiles;
  std::string LogosFiles;
  std::string OutputFiles;
};
void to_json(json &j, const FoldersInfo &fi);
void from_json(const json &j, FoldersInfo &fi);

/*
* NOTE: Optional: if the name is missing, I take the whole folder.
*/
struct FilesInfo 
{
  std::optional<std::string> InputName;
  std::string OutputName;
};
void to_json(json &j, const FilesInfo &mpi);
void from_json(const json &j, FilesInfo &mpi);


struct Configuration 
{
  FilesInfo filesInfo;
  FoldersInfo foldersInfo;
};
void to_json(json &j, const Configuration &config);
void from_json(const json &j, Configuration &config);


log4cxx::LoggerPtr initLogger(const std::string &loggerName = "MyAppLogger");

struct AppState {
  log4cxx::LoggerPtr logger;
  Configuration config;
};

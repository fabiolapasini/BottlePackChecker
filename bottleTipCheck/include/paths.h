#pragma once
#ifndef PATH_H
#define PATH_H

#include <filesystem>
#include <string>

// Get the current working directory
const std::filesystem::path cwd = std::filesystem::current_path();

// Define the project root (two levels up)
const std::filesystem::path project_root =
    cwd.parent_path().parent_path().parent_path().parent_path();

// Assets folder path
const std::filesystem::path assets_path = project_root / "Assets";

const std::filesystem::path check_tip_input = assets_path / "images";

const std::filesystem::path check_tip_output = assets_path / "output";

#endif    // PATH_H
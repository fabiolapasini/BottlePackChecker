

#include "../include/paths.h"


namespace Paths {

const std::filesystem::path cwd = std::filesystem::current_path();

const std::filesystem::path project_root =
    cwd.parent_path().parent_path().parent_path().parent_path();

const std::filesystem::path assets_path = project_root / "Assets";

const std::filesystem::path check_tip_input = assets_path / "images";

const std::filesystem::path check_tip_output = assets_path / "output";

}  // namespace Paths

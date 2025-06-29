

#include "../include/paths.h"


namespace Paths {

const std::filesystem::path cwd = std::filesystem::current_path();

const std::filesystem::path project_root =
    cwd.parent_path().parent_path().parent_path().parent_path();

const std::filesystem::path assets_path = project_root / "Assets";

// ----------------------------------------------------------------------

const std::filesystem::path depth_path = assets_path / "depths";

const std::filesystem::path image_path = assets_path / "images";

const std::filesystem::path logo_path = assets_path / "logos";

const std::filesystem::path output_path = assets_path / "output";

const std::filesystem::path pcl_path = assets_path / "pointcloud";

}  // namespace Paths

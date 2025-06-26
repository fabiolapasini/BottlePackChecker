# Add source to this project's executable.
add_executable(BottleTipCheck
    "main.cpp"
    "src/utils.cpp"
    "src/paths.cpp"
    "src/checkBottleCap.cpp"
)

# Find packages
find_package(fmt CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(log4cxx REQUIRED)

# Link libraries
target_link_libraries(BottleTipCheck
    PRIVATE
        fmt::fmt
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        log4cxx
        nlohmann_json::nlohmann_json
)

# Disable implicit conversion for json
set(nlohmann-json_IMPLICIT_CONVERSIONS OFF)

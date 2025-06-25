# Add source to this project's executable.
add_executable (BottleTipCheck  
    "main.cpp" 
    "src/utils.cpp"
    "src/paths.cpp"
    "src/checkBottleCap.cpp"
)


# Find Package
find_package(fmt CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

# Set OpenCVConfig.cmake Search Directory
set( OpenCV_DIR )
if( NOT CMAKE_CL_64 )
  set( OpenCV_DIR "C:/vcpkg/installed/x86-windows/share/opencv" )
else()
  set( OpenCV_DIR "C:/vcpkg/installed/x64-windows/share/opencv" )
endif()


if( OpenCV_FOUND )
  # Additional Include Directories
  include_directories( ${OpenCV_INCLUDE_DIRS} )

  # Additional Library Directories
  link_directories( ${OpenCV_LIB_DIR} )

  # Additional Dependencies
  target_link_libraries(BottleTipCheck 
  PRIVATE
  ${OpenCV_LIBS}
  )
endif()

find_package(nlohmann_json CONFIG REQUIRED)
find_package(log4cxx REQUIRED)

target_link_libraries(BottleTipCheck
  PUBLIC
    log4cxx
    nlohmann_json::nlohmann_json
)
set(nlohmann-json_IMPLICIT_CONVERSIONS OFF)

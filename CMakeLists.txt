cmake_minimum_required (VERSION 3.8)

if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

option(BUILD_TESTS "Build tests" ON)
message(STATUS "BUILD_TESTS: ${BUILD_TESTS}")
if(BUILD_TESTS)
    enable_testing()
    list(APPEND VCPKG_MANIFEST_FEATURES "test")
endif()

project ("BottleTipCheck")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

add_subdirectory(bottleTipCheck)
# add_subdirectory(test)

include(GNUInstallDirs)


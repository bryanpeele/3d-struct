cmake_minimum_required(VERSION 3.11)

include("cmake/HunterGate.cmake")
HunterGate(
  URL "https://github.com/ruslo/hunter/archive/v0.23.214.tar.gz"
  SHA1 "e14bc153a7f16d6a5eeec845fb0283c8fad8c358"
)

project(3dstruct)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE
          STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo")
endif()

option(ENABLE_CPPLINT "Turn on lint for style warnings" ON)
if(ENABLE_CPPLINT)
  set(CMAKE_CXX_CPPLINT cpplint)
endif()

hunter_add_package(Eigen)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Open3D 0.9.0.0 REQUIRED CONFIG)

include(GoogleTest)
enable_testing()

add_subdirectory(apps)
add_subdirectory(src/core)
add_subdirectory(src/geometry)
add_subdirectory(tests)
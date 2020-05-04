add_executable(app_open3d_example main.cpp)
target_link_libraries(app_open3d_example
  core
  ${Open3D_LIBRARIES}
)

target_include_directories(app_open3d_example
  PUBLIC
  ${Open3D_INCLUDE_DIRS}
)

if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
    target_compile_options(app_open3d_example PRIVATE -O3 -Wall -Wextra -Werror -D_GLIBCXX_USE_CXX11_ABI=0 -fPIC)
elseif(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    target_compile_options(app_open3d_example PRIVATE -Wall -Wextra -D_GLIBCXX_USE_CXX11_ABI=0 -fPIC)
endif()

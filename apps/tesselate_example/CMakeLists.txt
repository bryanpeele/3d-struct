add_executable(app_tessellate_example main.cpp)
target_link_libraries(app_tessellate_example
  core
  geometry
  ${Open3D_LIBRARIES}
)

target_include_directories(app_tessellate_example
  PUBLIC
  ${Open3D_INCLUDE_DIRS}
)

if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
  target_compile_options(app_tessellate_example PRIVATE -O3 -Wall -Wextra -Werror -D_GLIBCXX_USE_CXX11_ABI=0 -fPIC)
elseif(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  target_compile_options(app_tessellate_example PRIVATE -Wall -Wextra -D_GLIBCXX_USE_CXX11_ABI=0 -fPIC)
endif()

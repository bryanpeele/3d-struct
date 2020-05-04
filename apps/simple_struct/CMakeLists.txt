add_executable(app_simple_struct main.cpp)
target_link_libraries(app_simple_struct
  core
  geometry
  ${Open3D_LIBRARIES}
)

target_include_directories(app_simple_struct
  PUBLIC
  ${Open3D_INCLUDE_DIRS}
)

if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
    target_compile_options(app_simple_struct PRIVATE -O3 -Wall -Wextra -Werror -D_GLIBCXX_USE_CXX11_ABI=0 -fPIC)
elseif(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    target_compile_options(app_simple_struct PRIVATE -Wall -Wextra -D_GLIBCXX_USE_CXX11_ABI=0 -fPIC)
endif()

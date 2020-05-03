add_library(core
  src/util.cpp
)

target_link_libraries(core
  Eigen3::Eigen
)

target_include_directories(core PUBLIC
  "include"
)

if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
  target_compile_options(core PRIVATE -O3 -Wall -Wextra -Werror)
elseif(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    target_compile_options(core PRIVATE -Wall -Wextra)
endif()
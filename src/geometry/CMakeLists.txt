add_library(geometry SHARED
  src/graph.cpp
  src/math_util.cpp
  src/point.cpp
  src/structify.cpp
)

target_link_libraries(geometry
  core
)

target_include_directories(geometry PUBLIC
  "include"
)

if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
  target_compile_options(geometry PRIVATE -O3 -Wall -Wextra -Werror -D_GLIBCXX_USE_CXX11_ABI=0 -fPIC)
elseif(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  target_compile_options(geometry PRIVATE -Wall -Wextra -D_GLIBCXX_USE_CXX11_ABI=0 -fPIC)
endif()
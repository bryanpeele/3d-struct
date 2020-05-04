add_library(geometry SHARED
  src/point.cpp
  src/edge.cpp
  src/graph.cpp
  src/structify.cpp
)

target_link_libraries(geometry
  core
)

target_include_directories(geometry PUBLIC
  "include"
)

if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
  target_compile_options(geometry PRIVATE -O3 -Wall -Wextra -Werror)
elseif(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  target_compile_options(geometry PRIVATE -Wall -Wextra)
endif()
add_library(geometry
  src/point.cpp
  src/edge.cpp
)

target_link_libraries(geometry
  core
)

target_include_directories(geometry PUBLIC
  "include"
)

# target_compile_options(geometry PUBLIC "-fPIC")

if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
  target_compile_options(geometry PRIVATE -O3 -Wall -Wextra -Werror)
elseif(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  target_compile_options(geometry PRIVATE -Wall -Wextra)
endif()
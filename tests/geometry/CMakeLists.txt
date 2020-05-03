add_executable(point_test point_test.cpp)
target_link_libraries(point_test core geometry gtest gtest_main)
gtest_add_tests(TARGET point_test)

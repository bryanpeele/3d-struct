add_executable(util_test util_test.cpp)
target_link_libraries(util_test core gtest gtest_main)
gtest_add_tests(TARGET util_test)

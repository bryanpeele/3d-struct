#include "util.h"

#include <gtest/gtest.h>

TEST(Add, Add) {
  EXPECT_FLOAT_EQ(5.6, magnus::add(2.2, 3.4));
}

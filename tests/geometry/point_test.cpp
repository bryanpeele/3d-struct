#include "point.h"

#include <gtest/gtest.h>

#include "math.h"

TEST(Point, Point2d) {
  const double x = 12.0;
  const double y = 45.0;
  const auto coordinate = magnus::Vector2d(x, y);
  const auto point = magnus::geom::Point2d(coordinate);

  EXPECT_DOUBLE_EQ(x, point.x());
  EXPECT_DOUBLE_EQ(y, point.y());
}

TEST(Point, Point3d) {
  const double x = 12.0;
  const double y = 45.0;
  const double z = 45.0;
  const auto coordinate = magnus::Vector3d(x, y, z);
  const auto point = magnus::geom::Point3d(coordinate);

  EXPECT_DOUBLE_EQ(x, point.x());
  EXPECT_DOUBLE_EQ(y, point.y());
  EXPECT_DOUBLE_EQ(z, point.z());
}

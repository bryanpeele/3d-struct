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

TEST(Point, Point3dLessThan) {
  const auto point0 = magnus::geom::Point3d({1.0, 2.0, 3.0});
  const auto point1 = magnus::geom::Point3d({0.0, 5.0, 7.0});
  const auto point2 = magnus::geom::Point3d({1.0, 1.0, 2.0});
  const auto point3 = magnus::geom::Point3d({1.0, 2.0, 3.0});

  // 0 and 1
  EXPECT_FALSE(point0 < point1);
  EXPECT_TRUE(point1 < point0);

  // 0 and 2
  EXPECT_FALSE(point0 < point2);
  EXPECT_TRUE(point2 < point0);

  // 0 and 3
  EXPECT_FALSE(point0 < point3);
  EXPECT_FALSE(point3 < point0);

  // 1 and 2
  EXPECT_TRUE(point1 < point2);
  EXPECT_FALSE(point2 < point1);

  // 1 and 3
  EXPECT_TRUE(point1 < point3);
  EXPECT_FALSE(point3 < point1);

  // 2 and 3
  EXPECT_TRUE(point2 < point3);
  EXPECT_FALSE(point3 < point2);
}

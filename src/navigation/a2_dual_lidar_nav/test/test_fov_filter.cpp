#include <cmath>

#include "a2_dual_lidar_nav/fov_filter.hpp"
#include "gtest/gtest.h"

using a2_dual_lidar_nav::degreesToRadians;
using a2_dual_lidar_nav::withinHorizontalFov;

TEST(HorizontalFov, KeepsOnlyTheForwardHemisphereAt180Degrees)
{
  const double fov = degreesToRadians(180.0);
  EXPECT_TRUE(withinHorizontalFov(1.0, 0.0, 0.0, fov));
  EXPECT_TRUE(withinHorizontalFov(0.0, 1.0, 0.0, fov));
  EXPECT_TRUE(withinHorizontalFov(0.0, -1.0, 0.0, fov));
  EXPECT_FALSE(withinHorizontalFov(-1.0, 0.01, 0.0, fov));
  EXPECT_FALSE(withinHorizontalFov(-1.0, -0.01, 0.0, fov));
}

TEST(HorizontalFov, AppliesTheLimitInEachSensorFrame)
{
  const double fov = degreesToRadians(180.0);
  EXPECT_TRUE(withinHorizontalFov(1.0, 1.0, 0.0, fov));
  EXPECT_FALSE(withinHorizontalFov(-1.0, 1.0, 0.0, fov));
}

TEST(HorizontalFov, HandlesCentersAcrossTheAngleWrap)
{
  const double center = degreesToRadians(170.0);
  const double fov = degreesToRadians(60.0);
  EXPECT_TRUE(withinHorizontalFov(
    std::cos(degreesToRadians(-170.0)), std::sin(degreesToRadians(-170.0)), center, fov));
  EXPECT_FALSE(withinHorizontalFov(1.0, 0.0, center, fov));
}

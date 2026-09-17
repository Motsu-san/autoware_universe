// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::speed_scale_estimator
{

class UtilsTest : public ::testing::Test
{
protected:
  static rclcpp::Time to_time(double sec)
  {
    const auto s = static_cast<int32_t>(sec);
    return {s, static_cast<uint32_t>((sec - s) * 1e9), RCL_ROS_TIME};
  }

  static Imu create_imu_msg(double sec, double angular_velocity_z)
  {
    Imu msg;
    msg.header.stamp.sec = static_cast<int32_t>(sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>((sec - msg.header.stamp.sec) * 1e9);
    msg.angular_velocity.z = angular_velocity_z;
    return msg;
  }

  static VelocityReport create_velocity_msg(double sec, double velocity)
  {
    VelocityReport msg;
    msg.header.stamp.sec = static_cast<int32_t>(sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>((sec - msg.header.stamp.sec) * 1e9);
    msg.longitudinal_velocity = static_cast<float>(velocity);
    return msg;
  }

  static PoseStamped create_pose_msg(double sec, double x, double y)
  {
    PoseStamped msg;
    msg.header.stamp.sec = static_cast<int32_t>(sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>((sec - msg.header.stamp.sec) * 1e9);
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0.0;
    return msg;
  }

  static PoseStamped create_pose_msg(double sec, double x, double y, double z)
  {
    PoseStamped msg = create_pose_msg(sec, x, y);
    msg.pose.position.z = z;
    return msg;
  }

  /// Rotation about the y-axis. A positive pitch tilts the body x-axis downwards (ROS convention).
  static void set_pitch(PoseStamped & pose, double pitch)
  {
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = std::sin(0.5 * pitch);
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = std::cos(0.5 * pitch);
  }

  /// Rotation about the z-axis.
  static void set_yaw(PoseStamped & pose, double yaw)
  {
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
  }
};

TEST_F(UtilsTest, FindNearestImuReturnsClosestSample)
{
  const std::vector<Imu> imus = {
    create_imu_msg(0.0, 0.1), create_imu_msg(0.2, 0.3), create_imu_msg(0.5, 0.7)};

  const rclcpp::Time target_time(0, 210000000, RCL_ROS_TIME);
  const auto result = find_nearest_imu(imus, target_time);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->angular_velocity_z, 0.3, 1e-9);
  EXPECT_NEAR(result->stamp_diff, 0.01, 1e-9);
}

TEST_F(UtilsTest, FindNearestImuReturnsNulloptForEmptyInput)
{
  const std::vector<Imu> imus;
  const auto result = find_nearest_imu(imus, rclcpp::Time(1, 0));

  EXPECT_FALSE(result.has_value());
}

TEST_F(UtilsTest, FindNearestVelocityReportReturnsClosestSample)
{
  const std::vector<VelocityReport> velocity_reports = {
    create_velocity_msg(0.0, 1.0), create_velocity_msg(0.2, 3.0), create_velocity_msg(0.5, 7.0)};

  const rclcpp::Time target_time(0, 210000000, RCL_ROS_TIME);
  const auto result = find_nearest_velocity_report(velocity_reports, target_time);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->longitudinal_velocity, 3.0, 1e-9);
  EXPECT_NEAR(result->stamp_diff, 0.01, 1e-9);
}

TEST_F(UtilsTest, FindNearestVelocityReportReturnsNulloptForEmptyInput)
{
  const std::vector<VelocityReport> velocity_reports;
  const auto result = find_nearest_velocity_report(velocity_reports, rclcpp::Time(1, 0));

  EXPECT_FALSE(result.has_value());
}

TEST_F(UtilsTest, CalcPoseVelocityFromPoseDifference)
{
  const PoseStamped pose_a = create_pose_msg(0.0, 0.0, 0.0);
  const PoseStamped pose_b = create_pose_msg(0.1, 1.0, 0.0);

  EXPECT_NEAR(calc_pose_velocity(pose_a, pose_b), 10.0, 1e-9);
}

TEST_F(UtilsTest, CalcPoseVelocityIsUnbiasedForVerticalOffset)
{
  // A vertical offset perpendicular to the travel direction must not show up in the velocity. The
  // norm would return sqrt(1.0^2 + 0.1^2) / 0.1 = 10.0499, i.e. a one-sided inflation that biases
  // the scale factor upwards; the projection returns exactly 10.0.
  const PoseStamped pose_a = create_pose_msg(0.0, 0.0, 0.0, 0.0);
  const PoseStamped pose_b = create_pose_msg(0.1, 1.0, 0.0, 0.1);

  EXPECT_NEAR(calc_pose_velocity(pose_a, pose_b), 10.0, 1e-9);
}

TEST_F(UtilsTest, CalcPoseVelocityFollowsBodyAxisOnSlope)
{
  // Travelling 1.0 m along a 0.1 rad uphill grade must report the speed along the road surface
  // (10.0), not its horizontal projection (cos(0.1) * 10.0 = 9.95).
  constexpr double pitch = -0.1;  // negative pitch points the body x-axis upwards

  PoseStamped pose_a = create_pose_msg(0.0, 0.0, 0.0, 0.0);
  set_pitch(pose_a, pitch);
  PoseStamped pose_b = create_pose_msg(0.1, std::cos(pitch), 0.0, -std::sin(pitch));
  set_pitch(pose_b, pitch);

  EXPECT_NEAR(calc_pose_velocity(pose_a, pose_b), 10.0, 1e-9);
}

TEST_F(UtilsTest, CalcPoseVelocityProjectsOntoRotatedBodyAxis)
{
  // Heading east: the displacement is along +y, so the body x-axis must follow the orientation.
  constexpr double yaw = M_PI_2;

  PoseStamped pose_a = create_pose_msg(0.0, 0.0, 0.0, 0.0);
  set_yaw(pose_a, yaw);
  PoseStamped pose_b = create_pose_msg(0.1, 0.0, 1.0, 0.0);
  set_yaw(pose_b, yaw);

  EXPECT_NEAR(calc_pose_velocity(pose_a, pose_b), 10.0, 1e-9);
}

TEST_F(UtilsTest, CalcPoseVelocityIsNegativeForReverseMotion)
{
  // The wheel velocity report is signed, so the pose velocity must be too. A non-negative value
  // would pair a positive observation with a negative observation matrix and drive the scale
  // factor negative; a negative value is rejected by the min_speed constraint instead.
  const PoseStamped pose_a = create_pose_msg(0.0, 0.0, 0.0, 0.0);
  const PoseStamped pose_b = create_pose_msg(0.1, -1.0, 0.0, 0.0);

  EXPECT_NEAR(calc_pose_velocity(pose_a, pose_b), -10.0, 1e-9);
}

// The tolerance below is 1e-6 rather than 1e-9 because create_velocity_msg and to_time both
// truncate `(sec - floor(sec)) * 1e9` to whole nanoseconds, and values such as 0.3 land 1 ns short.
// That perturbs the window width by ~1e-9 s and the mean by a few 1e-9 m/s. Every case here
// separates its expected value from the wrong answer by at least 0.5 m/s, so 1e-6 discriminates
// with six orders of magnitude to spare.
TEST_F(UtilsTest, CalcMeanVelocityReportAveragesLinearRamp)
{
  // v(t) = 10.0 + 5.0 t. The mean of a linear ramp over [0.0, 0.3] is its midpoint value, 10.75.
  // find_nearest_velocity_report at t = 0.3 would return 11.5 instead - a 7 % difference, and
  // exactly the bias this function exists to remove.
  const std::vector<VelocityReport> velocity_reports = {
    create_velocity_msg(0.0, 10.0), create_velocity_msg(0.1, 10.5), create_velocity_msg(0.2, 11.0),
    create_velocity_msg(0.3, 11.5)};

  const auto result = calc_mean_velocity_report(velocity_reports, to_time(0.0), to_time(0.3));

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->longitudinal_velocity, 10.75, 1e-6);
  EXPECT_NEAR(result->begin_longitudinal_velocity, 10.0, 1e-6);
  EXPECT_NEAR(result->end_longitudinal_velocity, 11.5, 1e-6);
  EXPECT_NEAR(result->begin_stamp_diff, 0.0, 1e-6);
  EXPECT_NEAR(result->end_stamp_diff, 0.0, 1e-6);
}

TEST_F(UtilsTest, CalcMeanVelocityReportInterpolatesWindowEnds)
{
  // The window falls strictly between two samples, so the result comes entirely from the endpoint
  // interpolation: v(0.05) = 12.5 and v(0.10) = 15.0, giving 13.75. The window is deliberately
  // asymmetric so that snapping either end to its nearest sample (which would give 10.0) cannot
  // coincide with the correct answer.
  const std::vector<VelocityReport> velocity_reports = {
    create_velocity_msg(0.0, 10.0), create_velocity_msg(0.2, 20.0)};

  const auto result = calc_mean_velocity_report(velocity_reports, to_time(0.05), to_time(0.10));

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->longitudinal_velocity, 13.75, 1e-6);
  EXPECT_NEAR(result->begin_longitudinal_velocity, 12.5, 1e-6);
  EXPECT_NEAR(result->end_longitudinal_velocity, 15.0, 1e-6);
  EXPECT_NEAR(result->begin_stamp_diff, 0.05, 1e-6);
  EXPECT_NEAR(result->end_stamp_diff, 0.10, 1e-6);
}

TEST_F(UtilsTest, CalcMeanVelocityReportClampsOutsideSampleRange)
{
  // The window extends past both ends of the buffer. Clamping holds 8.0 over [0.0, 0.2] and 12.0
  // over [0.3, 0.4], giving (0.2*8.0 + 0.1*10.0 + 0.1*12.0) / 0.4 = 9.5. Linear extrapolation
  // would invent v(0.0) = 0.0 and v(0.4) = 16.0 and return 8.0, i.e. motion that was never
  // measured. The stamp differences are what lets the caller reject this window outright.
  const std::vector<VelocityReport> velocity_reports = {
    create_velocity_msg(0.2, 8.0), create_velocity_msg(0.3, 12.0)};

  const auto result = calc_mean_velocity_report(velocity_reports, to_time(0.0), to_time(0.4));

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->longitudinal_velocity, 9.5, 1e-6);
  EXPECT_NEAR(result->begin_longitudinal_velocity, 8.0, 1e-6);
  EXPECT_NEAR(result->end_longitudinal_velocity, 12.0, 1e-6);
  EXPECT_NEAR(result->begin_stamp_diff, 0.2, 1e-6);
  EXPECT_NEAR(result->end_stamp_diff, 0.1, 1e-6);
}

TEST_F(UtilsTest, CalcMeanVelocityReportWithSingleSampleReturnsThatSample)
{
  // Clamped at both ends, so the average degrades exactly to the old point-sample behaviour. This
  // is the path the estimator takes when a timer tick brings no new velocity report.
  const std::vector<VelocityReport> velocity_reports = {create_velocity_msg(0.2, 9.0)};

  const auto result = calc_mean_velocity_report(velocity_reports, to_time(0.0), to_time(0.4));

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->longitudinal_velocity, 9.0, 1e-6);
  EXPECT_NEAR(result->begin_stamp_diff, 0.2, 1e-6);
  EXPECT_NEAR(result->end_stamp_diff, 0.2, 1e-6);
}

TEST_F(UtilsTest, CalcMeanVelocityReportMatchesPointSampleAtConstantSpeed)
{
  // At constant speed the window mean and the point sample agree, so the fix is a no-op there and
  // only changes what the estimator sees while the vehicle accelerates.
  const std::vector<VelocityReport> velocity_reports = {
    create_velocity_msg(0.0, 12.0), create_velocity_msg(0.1, 12.0), create_velocity_msg(0.2, 12.0),
    create_velocity_msg(0.3, 12.0), create_velocity_msg(0.4, 12.0), create_velocity_msg(0.5, 12.0)};

  const auto result = calc_mean_velocity_report(velocity_reports, to_time(0.1), to_time(0.4));
  const auto nearest = find_nearest_velocity_report(velocity_reports, to_time(0.4));

  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(nearest.has_value());
  EXPECT_NEAR(result->longitudinal_velocity, 12.0, 1e-6);
  EXPECT_NEAR(result->longitudinal_velocity, nearest->longitudinal_velocity, 1e-6);
}

TEST_F(UtilsTest, CalcMeanVelocityReportWithDegenerateWindowFallsBackToWindowEnd)
{
  // A zero-width window has no interval to average. Returning the limit of the average as the
  // window shrinks keeps the value finite; a NaN here would propagate through the Kalman gain into
  // the covariance and no later update could recover from it.
  const std::vector<VelocityReport> velocity_reports = {
    create_velocity_msg(0.0, 10.0), create_velocity_msg(0.2, 20.0)};

  const auto result = calc_mean_velocity_report(velocity_reports, to_time(0.1), to_time(0.1));

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(std::isfinite(result->longitudinal_velocity));
  EXPECT_NEAR(result->longitudinal_velocity, 15.0, 1e-6);
}

TEST_F(UtilsTest, CalcMeanVelocityReportIgnoresDuplicateStamps)
{
  // A repeated stamp would contribute a zero-width trapezoid, and a stamp that goes backwards a
  // negative-width one, which can flip the sign of the mean and therefore of the observation
  // matrix. The result must match the duplicate-free sequence exactly.
  const std::vector<VelocityReport> velocity_reports = {
    create_velocity_msg(0.1, 10.0), create_velocity_msg(0.1, 10.0), create_velocity_msg(0.2, 12.0)};

  const auto result = calc_mean_velocity_report(velocity_reports, to_time(0.0), to_time(0.2));

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->longitudinal_velocity, 10.5, 1e-6);
}

TEST_F(UtilsTest, CalcMeanVelocityReportReturnsNulloptForEmptyInput)
{
  const std::vector<VelocityReport> velocity_reports;

  const auto result = calc_mean_velocity_report(velocity_reports, to_time(0.0), to_time(0.3));

  EXPECT_FALSE(result.has_value());
}

}  // namespace autoware::speed_scale_estimator

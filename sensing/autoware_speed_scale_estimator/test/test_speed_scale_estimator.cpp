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

#include "speed_scale_estimator.hpp"

#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace autoware::speed_scale_estimator
{

class SpeedScaleEstimatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    parameters_.update_interval = 0.1;
    parameters_.max_pose_lag = 0.5;
    parameters_.max_stamp_lag = 0.2;
    parameters_.sensor_buffer_duration = 1.0;
    parameters_.initial_speed_scale_factor = 1.0;
    parameters_.initial_speed_scale_factor_covariance = 1000.0;
    parameters_.process_noise_covariance = 0.01;
    parameters_.measurement_noise_covariance = 0.01;
    parameters_.max_angular_velocity = 0.0105;  // ~0.6 deg/s
    parameters_.max_speed = 17.0;
    parameters_.min_speed = 6.0;
    parameters_.enable_ndt_stability_check = true;
    parameters_.max_ndt_longitudinal_correction = 0.15;
    parameters_.max_ndt_lateral_correction = 0.15;
    parameters_.enable_degenerate_environment_check = false;
    parameters_.allowed_environment_ids = {0};

    reset_estimator();
  }

  void reset_estimator() { estimator_ = std::make_unique<SpeedScaleEstimator>(parameters_); }

  static PoseStamped create_pose_msg(double sec, double x, double y)
  {
    PoseStamped pose;
    pose.header.stamp.sec = static_cast<int32_t>(sec);
    pose.header.stamp.nanosec = static_cast<uint32_t>((sec - pose.header.stamp.sec) * 1e9);
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    return pose;
  }

  /// NDT initial-to-result relative pose: x is the longitudinal, y the lateral correction.
  static PoseStamped create_ndt_correction_msg(double sec, double longitudinal, double lateral)
  {
    return create_pose_msg(sec, longitudinal, lateral);
  }

  static Int32Stamped create_environment_id_msg(double sec, int32_t environment_id)
  {
    Int32Stamped msg;
    msg.stamp.sec = static_cast<int32_t>(sec);
    msg.stamp.nanosec = static_cast<uint32_t>((sec - msg.stamp.sec) * 1e9);
    msg.data = environment_id;
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

  static Imu create_imu_msg(double sec, double angular_velocity_z)
  {
    Imu msg;
    msg.header.stamp.sec = static_cast<int32_t>(sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>((sec - msg.header.stamp.sec) * 1e9);
    msg.angular_velocity.x = 0.0;
    msg.angular_velocity.y = 0.0;
    msg.angular_velocity.z = angular_velocity_z;
    return msg;
  }

  SpeedScaleEstimatorParameters parameters_;
  std::unique_ptr<SpeedScaleEstimator> estimator_;
};

TEST_F(SpeedScaleEstimatorTest, EstimationBehaviorTest)
{
  int successful_estimations = 0;
  double estimated_speed_scale_factor = 0.0;

  for (double t = 0.0; t <= 30.0; t += 0.1) {
    std::vector<PoseStamped> poses = {create_pose_msg(t, 10.0 * t, 0.0)};
    std::vector<VelocityReport> velocity_reports = {create_velocity_msg(t, 5.0)};
    std::vector<Imu> imus = {create_imu_msg(t, 0.005)};
    auto result = estimator_->update(
      poses, imus, velocity_reports, {create_ndt_correction_msg(t, 0.0, 0.0)}, {});

    if (result) {
      successful_estimations++;
      estimated_speed_scale_factor = result.value().estimated_speed_scale_factor;
    }
  }

  EXPECT_GT(successful_estimations, 0);
  EXPECT_NEAR(estimated_speed_scale_factor, 2.0, 0.5);
}

TEST_F(SpeedScaleEstimatorTest, InsufficientPoses)
{
  std::vector<PoseStamped> poses = {create_pose_msg(1.0, 0.0, 0.0)};
  std::vector<VelocityReport> velocity_reports = {create_velocity_msg(1.0, 5.0)};
  std::vector<Imu> imus = {create_imu_msg(1.0, 0.1)};

  auto result = estimator_->update(
    poses, imus, velocity_reports, {create_ndt_correction_msg(1.0, 0.0, 0.0)}, {});
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::WaitingForNextPose);
}

TEST_F(SpeedScaleEstimatorTest, AngularVelocityConstraintViolation)
{
  std::vector<PoseStamped> poses_t0 = {create_pose_msg(0.0, 0.0, 0.0)};
  std::vector<VelocityReport> velocity_reports_t0 = {create_velocity_msg(0.0, 5.0)};
  std::vector<Imu> imus_t0 = {create_imu_msg(0.0, 0.0)};
  (void)estimator_->update(
    poses_t0, imus_t0, velocity_reports_t0, {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  std::vector<PoseStamped> poses = {create_pose_msg(0.1, 0.5, 0.0)};
  std::vector<VelocityReport> velocity_reports = {create_velocity_msg(0.1, 5.0)};
  std::vector<Imu> imus = {create_imu_msg(0.1, 0.05)};

  const auto result = estimator_->update(
    poses, imus, velocity_reports, {create_ndt_correction_msg(0.1, 0.0, 0.0)}, {});

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::AngularVelocityTooHigh);
}

TEST_F(SpeedScaleEstimatorTest, SpeedConstraintViolation)
{
  std::vector<PoseStamped> poses_t0 = {create_pose_msg(0.0, 0.0, 0.0)};
  std::vector<VelocityReport> velocity_reports_t0 = {create_velocity_msg(0.0, 5.0)};
  std::vector<Imu> imus_t0 = {create_imu_msg(0.0, 0.0)};
  (void)estimator_->update(
    poses_t0, imus_t0, velocity_reports_t0, {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  std::vector<PoseStamped> poses = {create_pose_msg(0.1, 0.1, 0.0)};
  std::vector<VelocityReport> velocity_reports = {create_velocity_msg(0.1, 1.0)};
  std::vector<Imu> imus = {create_imu_msg(0.1, 0.005)};

  const auto result = estimator_->update(
    poses, imus, velocity_reports, {create_ndt_correction_msg(0.1, 0.0, 0.0)}, {});

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::VelocityTooLow);
}

TEST_F(SpeedScaleEstimatorTest, VelocityReportTimestampMismatch)
{
  std::vector<PoseStamped> poses_t0 = {create_pose_msg(0.0, 0.0, 0.0)};
  std::vector<VelocityReport> velocity_reports_t0 = {create_velocity_msg(2.0, 5.0)};
  std::vector<Imu> imus_t0 = {create_imu_msg(0.0, 0.0)};
  (void)estimator_->update(
    poses_t0, imus_t0, velocity_reports_t0, {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  std::vector<PoseStamped> poses = {create_pose_msg(0.2, 2.0, 0.0)};
  std::vector<VelocityReport> velocity_reports = {create_velocity_msg(2.0, 5.0)};
  std::vector<Imu> imus = {create_imu_msg(0.2, 0.0)};

  const auto result = estimator_->update(
    poses, imus, velocity_reports, {create_ndt_correction_msg(0.2, 0.0, 0.0)}, {});

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::VelocityReportTimestampMismatch);
}

TEST_F(SpeedScaleEstimatorTest, VelocityReportTimestampOffsetWithinTolerance)
{
  std::vector<PoseStamped> poses_t0 = {create_pose_msg(0.0, 0.0, 0.0)};
  std::vector<VelocityReport> velocity_reports_t0 = {create_velocity_msg(0.0, 5.0)};
  std::vector<Imu> imus_t0 = {create_imu_msg(0.0, 0.0)};
  (void)estimator_->update(
    poses_t0, imus_t0, velocity_reports_t0, {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  std::vector<PoseStamped> poses = {create_pose_msg(0.2, 2.0, 0.0)};
  std::vector<VelocityReport> velocity_reports = {create_velocity_msg(0.344, 5.0)};
  std::vector<Imu> imus = {create_imu_msg(0.344, 0.005)};

  const auto result = estimator_->update(
    poses, imus, velocity_reports, {create_ndt_correction_msg(0.2, 0.0, 0.0)}, {});

  ASSERT_TRUE(result);
}

TEST_F(SpeedScaleEstimatorTest, PoseLagWithinTolerance)
{
  std::vector<PoseStamped> poses_t0 = {create_pose_msg(0.0, 0.0, 0.0)};
  std::vector<VelocityReport> velocity_reports_t0 = {create_velocity_msg(0.0, 5.0)};
  std::vector<Imu> imus_t0 = {create_imu_msg(0.0, 0.0)};
  (void)estimator_->update(
    poses_t0, imus_t0, velocity_reports_t0, {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  std::vector<PoseStamped> poses = {create_pose_msg(0.2, 2.0, 0.0)};
  std::vector<VelocityReport> velocity_reports = {create_velocity_msg(0.2, 5.0)};
  std::vector<Imu> imus = {create_imu_msg(0.2, 0.005)};

  const auto result = estimator_->update(
    poses, imus, velocity_reports, {create_ndt_correction_msg(0.2, 0.0, 0.0)}, {});

  ASSERT_TRUE(result);
}

TEST_F(SpeedScaleEstimatorTest, SensorBufferRetentionAcrossUpdates)
{
  std::vector<PoseStamped> poses_t0 = {create_pose_msg(0.0, 0.0, 0.0)};
  std::vector<VelocityReport> velocity_reports_t0 = {create_velocity_msg(0.0, 5.0)};
  std::vector<Imu> imus_t0 = {create_imu_msg(0.0, 0.0)};
  (void)estimator_->update(
    poses_t0, imus_t0, velocity_reports_t0, {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  std::vector<PoseStamped> poses = {create_pose_msg(0.2, 2.0, 0.0)};
  const std::vector<VelocityReport> empty_velocity;
  const std::vector<Imu> empty_imu;

  const auto result = estimator_->update(
    poses, empty_imu, empty_velocity, {create_ndt_correction_msg(0.2, 0.0, 0.0)}, {});

  ASSERT_TRUE(result);
}

TEST_F(SpeedScaleEstimatorTest, WheelVelocityIsAveragedOverPoseInterval)
{
  // v(t) = 10.0 + 5.0 t, x(t) = 10.0 t + 2.5 t^2. Over [0.0, 0.2] the pose velocity is
  // x(0.2) / 0.2 = 10.5 m/s, i.e. v at the window midpoint. The wheel sample at t = 0.2 is
  // 11.0 m/s, so pairing the two would report a scale factor of 10.5 / 11.0 = 0.9545 for a
  // vehicle whose wheel speed is in fact exact. Averaging over the window makes the innovation
  // exactly zero, so the estimate stays at 1.0 regardless of the Kalman gain.
  (void)estimator_->update(
    {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.0, 10.0)},
    {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  const auto result = estimator_->update(
    {create_pose_msg(0.2, 2.1, 0.0)}, {create_imu_msg(0.2, 0.005)},
    {create_velocity_msg(0.05, 10.25), create_velocity_msg(0.1, 10.5),
     create_velocity_msg(0.15, 10.75), create_velocity_msg(0.2, 11.0)},
    {create_ndt_correction_msg(0.2, 0.0, 0.0)}, {});

  ASSERT_TRUE(result);
  EXPECT_NEAR(result.value().velocity_from_pose, 10.5, 1e-6);
  EXPECT_NEAR(result.value().velocity_from_wheel, 10.5, 1e-6);
  EXPECT_NEAR(result.value().estimated_speed_scale_factor, 1.0, 1e-6);
}

TEST_F(SpeedScaleEstimatorTest, WheelVelocityAverageIncludesSamplesInsidePoseInterval)
{
  // Both ends of the interval read 10.0 m/s; only the sample in between is different, so a lookup
  // at either end cannot see the motion that actually produced the displacement. This mirrors
  // NdtCorrectionUsesWorstSampleWithinPoseInterval for the wheel velocity.
  (void)estimator_->update(
    {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.0, 10.0)},
    {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  const auto result = estimator_->update(
    {create_pose_msg(0.2, 2.4, 0.0)}, {create_imu_msg(0.2, 0.005)},
    {create_velocity_msg(0.1, 14.0), create_velocity_msg(0.2, 10.0)},
    {create_ndt_correction_msg(0.2, 0.0, 0.0)}, {});

  ASSERT_TRUE(result);
  EXPECT_NEAR(result.value().velocity_from_wheel, 12.0, 1e-6);
  EXPECT_NEAR(result.value().estimated_speed_scale_factor, 1.0, 1e-6);
}

TEST_F(SpeedScaleEstimatorTest, VelocityReportDoesNotCoverPoseIntervalBegin)
{
  // Only the far end of the interval has a wheel sample, so the average would be a clamped hold
  // over 0.4 s rather than a measurement of the interval the pose velocity came from. The seeding
  // call has to carry a velocity report because the empty-buffer check precedes the previous-pose
  // latch, so an empty velocity list would never latch the pose at all.
  (void)estimator_->update(
    {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.4, 10.0)},
    {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  const auto result = estimator_->update(
    {create_pose_msg(0.4, 4.0, 0.0)}, {create_imu_msg(0.4, 0.005)},
    {create_velocity_msg(0.4, 10.0)},
    {create_ndt_correction_msg(0.2, 0.0, 0.0), create_ndt_correction_msg(0.4, 0.0, 0.0)}, {});

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::VelocityReportTimestampMismatch);
  EXPECT_NEAR(result.error().context.stamp_diff, 0.4, 1e-6);
  EXPECT_DOUBLE_EQ(result.error().context.stamp_diff_threshold, parameters_.max_stamp_lag);
}

TEST_F(SpeedScaleEstimatorTest, VelocityReportDoesNotCoverPoseIntervalEnd)
{
  // Symmetric to the previous case: the buffer only reaches the beginning of the interval.
  (void)estimator_->update(
    {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.0, 10.0)},
    {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  const auto result = estimator_->update(
    {create_pose_msg(0.4, 4.0, 0.0)}, {create_imu_msg(0.4, 0.005)}, {},
    {create_ndt_correction_msg(0.2, 0.0, 0.0), create_ndt_correction_msg(0.4, 0.0, 0.0)}, {});

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::VelocityReportTimestampMismatch);
  EXPECT_NEAR(result.error().context.stamp_diff, 0.4, 1e-6);
}

TEST_F(SpeedScaleEstimatorTest, EmptyDataHandling)
{
  std::vector<PoseStamped> poses = {create_pose_msg(1.0, 0.0, 0.0)};
  std::vector<VelocityReport> empty_velocity;
  std::vector<Imu> empty_imu;

  auto result = estimator_->update(poses, empty_imu, empty_velocity, {}, {});
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::ImuEmpty);
}

// --- NDT scan matching stability gate ---

TEST_F(SpeedScaleEstimatorTest, NdtLongitudinalCorrectionTooLarge)
{
  (void)estimator_->update(
    {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.0, 5.0)},
    {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  const auto result = estimator_->update(
    {create_pose_msg(0.2, 2.0, 0.0)}, {create_imu_msg(0.2, 0.005)}, {create_velocity_msg(0.2, 5.0)},
    {create_ndt_correction_msg(0.2, 0.2, 0.0)}, {});

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::NdtCorrectionTooLarge);
  EXPECT_DOUBLE_EQ(result.error().context.ndt_longitudinal_correction, 0.2);
}

TEST_F(SpeedScaleEstimatorTest, NdtLateralCorrectionTooLarge)
{
  (void)estimator_->update(
    {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.0, 5.0)},
    {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  const auto result = estimator_->update(
    {create_pose_msg(0.2, 2.0, 0.0)}, {create_imu_msg(0.2, 0.005)}, {create_velocity_msg(0.2, 5.0)},
    {create_ndt_correction_msg(0.2, 0.0, -0.2)}, {});

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::NdtCorrectionTooLarge);
  EXPECT_DOUBLE_EQ(result.error().context.ndt_lateral_correction, 0.2);
}

TEST_F(SpeedScaleEstimatorTest, NdtCorrectionUsesWorstSampleWithinPoseInterval)
{
  // Both ends of the interval look clean; only a sample in between exceeds the threshold.
  (void)estimator_->update(
    {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.0, 5.0)},
    {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  const auto result = estimator_->update(
    {create_pose_msg(0.2, 2.0, 0.0)}, {create_imu_msg(0.2, 0.005)}, {create_velocity_msg(0.2, 5.0)},
    {create_ndt_correction_msg(0.1, 0.5, 0.0), create_ndt_correction_msg(0.2, 0.0, 0.0)}, {});

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::NdtCorrectionTooLarge);
  EXPECT_DOUBLE_EQ(result.error().context.ndt_longitudinal_correction, 0.5);
}

TEST_F(SpeedScaleEstimatorTest, NdtStabilityUnavailableWhenBufferEmpty)
{
  (void)estimator_->update(
    {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.0, 5.0)},
    {}, {});

  const auto result = estimator_->update(
    {create_pose_msg(0.2, 2.0, 0.0)}, {create_imu_msg(0.2, 0.005)}, {create_velocity_msg(0.2, 5.0)},
    {}, {});

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::NdtStabilityUnavailable);
}

TEST_F(SpeedScaleEstimatorTest, NdtStabilityUnavailableWhenIntervalNotCovered)
{
  // Only the far end of the interval has a sample, so the window is not actually covered.
  (void)estimator_->update(
    {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.0, 5.0)},
    {}, {});

  const auto result = estimator_->update(
    {create_pose_msg(0.4, 4.0, 0.0)}, {create_imu_msg(0.4, 0.005)}, {create_velocity_msg(0.4, 5.0)},
    {create_ndt_correction_msg(0.4, 0.0, 0.0)}, {});

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::NdtStabilityUnavailable);
  EXPECT_DOUBLE_EQ(result.error().context.stamp_diff_threshold, parameters_.max_stamp_lag);
}

TEST_F(SpeedScaleEstimatorTest, NdtStabilityCheckDisabledIgnoresLargeCorrection)
{
  parameters_.enable_ndt_stability_check = false;
  reset_estimator();

  (void)estimator_->update(
    {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.0, 5.0)},
    {create_ndt_correction_msg(0.0, 10.0, 10.0)}, {});

  const auto result = estimator_->update(
    {create_pose_msg(0.2, 2.0, 0.0)}, {create_imu_msg(0.2, 0.005)}, {create_velocity_msg(0.2, 5.0)},
    {create_ndt_correction_msg(0.2, 10.0, 10.0)}, {});

  ASSERT_TRUE(result);
  EXPECT_DOUBLE_EQ(result.value().ndt_longitudinal_correction, 0.0);
}

// --- Degenerate environment gate ---

TEST_F(SpeedScaleEstimatorTest, DegenerateEnvironmentRejected)
{
  parameters_.enable_degenerate_environment_check = true;

  for (const int32_t degenerate_id : {1, 2}) {
    reset_estimator();

    (void)estimator_->update(
      {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.0, 5.0)},
      {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {create_environment_id_msg(0.0, 0)});

    const auto result = estimator_->update(
      {create_pose_msg(0.2, 2.0, 0.0)}, {create_imu_msg(0.2, 0.005)},
      {create_velocity_msg(0.2, 5.0)}, {create_ndt_correction_msg(0.2, 0.0, 0.0)},
      {create_environment_id_msg(0.2, degenerate_id)});

    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().reason, UpdateFailureReason::DegenerateEnvironment);
    EXPECT_EQ(result.error().context.environment_id, degenerate_id);
  }
}

TEST_F(SpeedScaleEstimatorTest, AllowedEnvironmentAccepted)
{
  parameters_.enable_degenerate_environment_check = true;
  reset_estimator();

  (void)estimator_->update(
    {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.0, 5.0)},
    {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {create_environment_id_msg(0.0, 0)});

  const auto result = estimator_->update(
    {create_pose_msg(0.2, 2.0, 0.0)}, {create_imu_msg(0.2, 0.005)}, {create_velocity_msg(0.2, 5.0)},
    {create_ndt_correction_msg(0.2, 0.0, 0.0)}, {create_environment_id_msg(0.2, 0)});

  ASSERT_TRUE(result);
  EXPECT_EQ(result.value().environment_id, 0);
}

TEST_F(SpeedScaleEstimatorTest, EnvironmentIdUnavailableWhenNeverReceived)
{
  parameters_.enable_degenerate_environment_check = true;
  reset_estimator();

  (void)estimator_->update(
    {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.0, 5.0)},
    {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {});

  const auto result = estimator_->update(
    {create_pose_msg(0.2, 2.0, 0.0)}, {create_imu_msg(0.2, 0.005)}, {create_velocity_msg(0.2, 5.0)},
    {create_ndt_correction_msg(0.2, 0.0, 0.0)}, {});

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::EnvironmentIdUnavailable);
}

TEST_F(SpeedScaleEstimatorTest, StaleEnvironmentIdSkipsEstimation)
{
  parameters_.enable_degenerate_environment_check = true;
  reset_estimator();

  (void)estimator_->update(
    {create_pose_msg(1.0, 0.0, 0.0)}, {create_imu_msg(1.0, 0.0)}, {create_velocity_msg(1.0, 5.0)},
    {create_ndt_correction_msg(1.0, 0.0, 0.0)}, {});

  const auto result = estimator_->update(
    {create_pose_msg(1.5, 5.0, 0.0)}, {create_imu_msg(1.5, 0.005)}, {create_velocity_msg(1.5, 5.0)},
    {create_ndt_correction_msg(1.5, 0.0, 0.0)}, {create_environment_id_msg(1.0, 0)});

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::EnvironmentIdUnavailable);
  EXPECT_DOUBLE_EQ(result.error().context.stamp_diff_threshold, parameters_.max_stamp_lag);
}

TEST_F(SpeedScaleEstimatorTest, DegenerateEnvironmentCheckDisabledIgnoresEnvironmentId)
{
  // Mirrors the default configuration, where environment_adaptor is not launched.
  (void)estimator_->update(
    {create_pose_msg(0.0, 0.0, 0.0)}, {create_imu_msg(0.0, 0.0)}, {create_velocity_msg(0.0, 5.0)},
    {create_ndt_correction_msg(0.0, 0.0, 0.0)}, {create_environment_id_msg(0.0, 1)});

  const auto result = estimator_->update(
    {create_pose_msg(0.2, 2.0, 0.0)}, {create_imu_msg(0.2, 0.005)}, {create_velocity_msg(0.2, 5.0)},
    {create_ndt_correction_msg(0.2, 0.0, 0.0)}, {create_environment_id_msg(0.2, 1)});

  ASSERT_TRUE(result);
  EXPECT_EQ(result.value().environment_id, 0);
}

}  // namespace autoware::speed_scale_estimator

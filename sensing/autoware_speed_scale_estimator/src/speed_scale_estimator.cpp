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

#include "utils.hpp"

#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace autoware::speed_scale_estimator
{

SpeedScaleEstimator::SpeedScaleEstimator(const SpeedScaleEstimatorParameters & parameters)
: parameters_(parameters),
  estimated_speed_scale_factor_(parameters.initial_speed_scale_factor),
  covariance_(parameters.initial_speed_scale_factor_covariance)
{
}

double SpeedScaleEstimator::get_update_interval_sec() const
{
  return parameters_.update_interval;
}

SpeedScaleEstimatorNotUpdated SpeedScaleEstimator::make_not_updated(
  const UpdateFailureReason reason, const UpdateFailureContext & context) const
{
  return {reason, context, estimated_speed_scale_factor_};
}

void SpeedScaleEstimator::update_imu_buffer(const std::vector<Imu> & imus)
{
  for (const auto & imu : imus) {
    if (
      !imu_buffer_.empty() &&
      rclcpp::Time(imu.header.stamp) < rclcpp::Time(imu_buffer_.back().header.stamp)) {
      continue;
    }
    imu_buffer_.emplace_back(imu);
  }

  while (!imu_buffer_.empty() && (rclcpp::Time(imu_buffer_.back().header.stamp) -
                                  rclcpp::Time(imu_buffer_.front().header.stamp))
                                     .seconds() > parameters_.sensor_buffer_duration) {
    imu_buffer_.pop_front();
  }
}

void SpeedScaleEstimator::update_velocity_report_buffer(
  const std::vector<VelocityReport> & velocity_reports)
{
  for (const auto & velocity_report : velocity_reports) {
    if (
      !velocity_report_buffer_.empty() &&
      rclcpp::Time(velocity_report.header.stamp) <
        rclcpp::Time(velocity_report_buffer_.back().header.stamp)) {
      continue;
    }
    velocity_report_buffer_.emplace_back(velocity_report);
  }

  while (!velocity_report_buffer_.empty() &&
         (rclcpp::Time(velocity_report_buffer_.back().header.stamp) -
          rclcpp::Time(velocity_report_buffer_.front().header.stamp))
             .seconds() > parameters_.sensor_buffer_duration) {
    velocity_report_buffer_.pop_front();
  }
}

void SpeedScaleEstimator::update_ndt_correction_buffer(
  const std::vector<PoseStamped> & ndt_corrections)
{
  for (const auto & ndt_correction : ndt_corrections) {
    if (
      !ndt_correction_buffer_.empty() &&
      rclcpp::Time(ndt_correction.header.stamp) <
        rclcpp::Time(ndt_correction_buffer_.back().header.stamp)) {
      continue;
    }
    ndt_correction_buffer_.emplace_back(ndt_correction);
  }

  while (!ndt_correction_buffer_.empty() &&
         (rclcpp::Time(ndt_correction_buffer_.back().header.stamp) -
          rclcpp::Time(ndt_correction_buffer_.front().header.stamp))
             .seconds() > parameters_.sensor_buffer_duration) {
    ndt_correction_buffer_.pop_front();
  }
}

void SpeedScaleEstimator::update_environment_id_buffer(
  const std::vector<Int32Stamped> & environment_ids)
{
  for (const auto & environment_id : environment_ids) {
    if (
      !environment_id_buffer_.empty() &&
      rclcpp::Time(environment_id.stamp) < rclcpp::Time(environment_id_buffer_.back().stamp)) {
      continue;
    }
    environment_id_buffer_.emplace_back(environment_id);
  }

  while (!environment_id_buffer_.empty() && (rclcpp::Time(environment_id_buffer_.back().stamp) -
                                             rclcpp::Time(environment_id_buffer_.front().stamp))
                                                .seconds() > parameters_.sensor_buffer_duration) {
    environment_id_buffer_.pop_front();
  }
}

tl::expected<SpeedScaleEstimatorUpdated, SpeedScaleEstimatorNotUpdated> SpeedScaleEstimator::update(
  const std::vector<PoseStamped> & poses, const std::vector<Imu> & imus,
  const std::vector<VelocityReport> & velocity_reports,
  const std::vector<PoseStamped> & ndt_corrections,
  const std::vector<Int32Stamped> & environment_ids)
{
  update_imu_buffer(imus);
  update_velocity_report_buffer(velocity_reports);
  if (parameters_.enable_ndt_stability_check) {
    update_ndt_correction_buffer(ndt_corrections);
  }
  if (parameters_.enable_degenerate_environment_check) {
    update_environment_id_buffer(environment_ids);
  }

  if (poses.empty()) {
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::PoseEmpty));
  }

  if (imu_buffer_.empty()) {
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::ImuEmpty));
  }

  if (velocity_report_buffer_.empty()) {
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::VelocityReportEmpty));
  }

  const auto & pose_curr = poses.back();

  if (!previous_pose_) {
    previous_pose_ = pose_curr;
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::WaitingForNextPose));
  }

  const auto & pose_prev = previous_pose_.value();

  const double time_diff = calc_time_diff(pose_prev, pose_curr);
  if (time_diff > parameters_.max_pose_lag) {
    previous_pose_ = pose_curr;
    UpdateFailureContext context;
    context.time_diff = time_diff;
    context.time_diff_threshold = parameters_.max_pose_lag;
    return tl::make_unexpected(
      make_not_updated(UpdateFailureReason::TimeDifferenceTooLarge, context));
  }

  const double v_pose = calc_pose_velocity(pose_prev, pose_curr);

  const rclcpp::Time pose_time(pose_curr.header.stamp);
  const rclcpp::Time pose_prev_time(pose_prev.header.stamp);

  // v_pose is the mean velocity over [t_prev, t_curr], not the instantaneous velocity at t_curr, so
  // the wheel velocity it is paired with has to be the mean over the same window. A single sample
  // at t_curr sits half a window later and biases the ratio by -a * dt / (2 * v_wheel), which is
  // ~0.5 % at 0.5 m/s^2 and 16 m/s and, on a fixed route, repeats in the same places on every pass.
  const std::vector<VelocityReport> buffered_velocity_reports(
    velocity_report_buffer_.begin(), velocity_report_buffer_.end());
  const auto mean_velocity_report =
    calc_mean_velocity_report(buffered_velocity_reports, pose_prev_time, pose_time);
  if (!mean_velocity_report) {
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::VelocityReportEmpty));
  }

  // Both ends of the window enter the average, so require the buffer to actually reach them - the
  // same requirement the NDT correction gate imposes below, for the same reason.
  if (
    mean_velocity_report->begin_stamp_diff > parameters_.max_stamp_lag ||
    mean_velocity_report->end_stamp_diff > parameters_.max_stamp_lag) {
    UpdateFailureContext context;
    context.stamp_diff =
      std::max(mean_velocity_report->begin_stamp_diff, mean_velocity_report->end_stamp_diff);
    context.stamp_diff_threshold = parameters_.max_stamp_lag;
    return tl::make_unexpected(
      make_not_updated(UpdateFailureReason::VelocityReportTimestampMismatch, context));
  }

  const double v_wheel = mean_velocity_report->longitudinal_velocity;

  // Diagnostics only. The point sample is what the observation used to take, so publishing both
  // makes the correction verifiable straight from debug_info; the acceleration is what drove the
  // difference between them.
  const auto nearest_velocity_report =
    find_nearest_velocity_report(buffered_velocity_reports, pose_time);
  const double v_wheel_point =
    nearest_velocity_report ? nearest_velocity_report->longitudinal_velocity : v_wheel;
  const double longitudinal_acceleration = (mean_velocity_report->end_longitudinal_velocity -
                                            mean_velocity_report->begin_longitudinal_velocity) /
                                           time_diff;

  const std::vector<Imu> buffered_imus(imu_buffer_.begin(), imu_buffer_.end());
  const auto nearest_imu = find_nearest_imu(buffered_imus, pose_time);
  if (!nearest_imu) {
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::ImuEmpty));
  }

  if (nearest_imu->stamp_diff > parameters_.max_stamp_lag) {
    UpdateFailureContext context;
    context.stamp_diff = nearest_imu->stamp_diff;
    context.stamp_diff_threshold = parameters_.max_stamp_lag;
    return tl::make_unexpected(
      make_not_updated(UpdateFailureReason::ImuTimestampMismatch, context));
  }

  const double angular_velocity = nearest_imu->angular_velocity_z;
  if (std::abs(angular_velocity) > parameters_.max_angular_velocity) {
    UpdateFailureContext context;
    context.angular_velocity = angular_velocity;
    context.max_angular_velocity = parameters_.max_angular_velocity;
    return tl::make_unexpected(
      make_not_updated(UpdateFailureReason::AngularVelocityTooHigh, context));
  }

  if (v_pose > parameters_.max_speed) {
    UpdateFailureContext context;
    context.velocity = v_pose;
    context.velocity_threshold = parameters_.max_speed;
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::VelocityTooHigh, context));
  }

  if (v_pose < parameters_.min_speed) {
    UpdateFailureContext context;
    context.velocity = v_pose;
    context.velocity_threshold = parameters_.min_speed;
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::VelocityTooLow, context));
  }

  if (std::abs(v_wheel) < 1e-6) {
    UpdateFailureContext context;
    context.velocity = v_wheel;
    return tl::make_unexpected(
      make_not_updated(UpdateFailureReason::VelocityReportTooSmall, context));
  }

  // NDT scan matching stability: reject the sample when NDT had to move its initial guess by a
  // large amount anywhere in the window used to derive the pose velocity.
  double ndt_longitudinal_correction = 0.0;
  double ndt_lateral_correction = 0.0;
  if (parameters_.enable_ndt_stability_check) {
    const std::vector<PoseStamped> buffered_ndt_corrections(
      ndt_correction_buffer_.begin(), ndt_correction_buffer_.end());
    const auto worst_ndt_correction =
      find_worst_ndt_correction(buffered_ndt_corrections, pose_prev_time, pose_time);
    if (!worst_ndt_correction) {
      return tl::make_unexpected(make_not_updated(UpdateFailureReason::NdtStabilityUnavailable));
    }

    if (
      worst_ndt_correction->begin_stamp_diff > parameters_.max_stamp_lag ||
      worst_ndt_correction->end_stamp_diff > parameters_.max_stamp_lag) {
      UpdateFailureContext context;
      context.stamp_diff =
        std::max(worst_ndt_correction->begin_stamp_diff, worst_ndt_correction->end_stamp_diff);
      context.stamp_diff_threshold = parameters_.max_stamp_lag;
      return tl::make_unexpected(
        make_not_updated(UpdateFailureReason::NdtStabilityUnavailable, context));
    }

    ndt_longitudinal_correction = worst_ndt_correction->longitudinal;
    ndt_lateral_correction = worst_ndt_correction->lateral;

    if (
      ndt_longitudinal_correction > parameters_.max_ndt_longitudinal_correction ||
      ndt_lateral_correction > parameters_.max_ndt_lateral_correction) {
      UpdateFailureContext context;
      context.ndt_longitudinal_correction = ndt_longitudinal_correction;
      context.ndt_lateral_correction = ndt_lateral_correction;
      context.ndt_longitudinal_correction_threshold = parameters_.max_ndt_longitudinal_correction;
      context.ndt_lateral_correction_threshold = parameters_.max_ndt_lateral_correction;
      return tl::make_unexpected(
        make_not_updated(UpdateFailureReason::NdtCorrectionTooLarge, context));
    }
  }

  // Degenerate environment: NDT slips longitudinally in feature-poor or uniform areas, so the pose
  // velocity cannot be trusted there.
  int32_t environment_id = 0;
  if (parameters_.enable_degenerate_environment_check) {
    const std::vector<Int32Stamped> buffered_environment_ids(
      environment_id_buffer_.begin(), environment_id_buffer_.end());
    const auto nearest_environment_id =
      find_nearest_environment_id(buffered_environment_ids, pose_time);
    if (!nearest_environment_id) {
      return tl::make_unexpected(make_not_updated(UpdateFailureReason::EnvironmentIdUnavailable));
    }

    if (nearest_environment_id->stamp_diff > parameters_.max_stamp_lag) {
      UpdateFailureContext context;
      context.stamp_diff = nearest_environment_id->stamp_diff;
      context.stamp_diff_threshold = parameters_.max_stamp_lag;
      return tl::make_unexpected(
        make_not_updated(UpdateFailureReason::EnvironmentIdUnavailable, context));
    }

    environment_id = nearest_environment_id->environment_id;

    const auto & allowed_ids = parameters_.allowed_environment_ids;
    if (std::find(allowed_ids.begin(), allowed_ids.end(), environment_id) == allowed_ids.end()) {
      UpdateFailureContext context;
      context.environment_id = environment_id;
      return tl::make_unexpected(
        make_not_updated(UpdateFailureReason::DegenerateEnvironment, context));
    }
  }

  const double z = v_pose;

  const double x_pred = estimated_speed_scale_factor_;
  const double P_pred = covariance_ + parameters_.process_noise_covariance;  // NOLINT

  const double H = v_wheel;  // NOLINT
  const double innovation = z - H * x_pred;
  const double S = H * H * P_pred + parameters_.measurement_noise_covariance;  // NOLINT
  const double K = (P_pred * H) / S;                                           // NOLINT

  estimated_speed_scale_factor_ = x_pred + K * innovation;
  covariance_ = (1.0 - K * H) * P_pred;

  previous_pose_ = pose_curr;

  SpeedScaleEstimatorUpdated result;
  result.estimated_speed_scale_factor = estimated_speed_scale_factor_;
  result.covariance = covariance_;
  result.velocity_from_pose = v_pose;
  result.velocity_from_wheel = v_wheel;
  result.kalman_gain = K;
  result.time_diff = time_diff;
  result.ndt_longitudinal_correction = ndt_longitudinal_correction;
  result.ndt_lateral_correction = ndt_lateral_correction;
  result.environment_id = environment_id;
  result.velocity_from_wheel_point = v_wheel_point;
  result.longitudinal_acceleration = longitudinal_acceleration;

  return result;
}

}  // namespace autoware::speed_scale_estimator

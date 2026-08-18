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

#include <geometry_msgs/msg/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace autoware::speed_scale_estimator
{
namespace
{

/// Body x-axis of a pose, expressed in the frame the pose is given in.
std::array<double, 3> body_x_axis(const geometry_msgs::msg::Quaternion & q)
{
  return {
    1.0 - 2.0 * (q.y * q.y + q.z * q.z), 2.0 * (q.x * q.y + q.w * q.z),
    2.0 * (q.x * q.z - q.w * q.y)};
}

}  // namespace

double calc_time_diff(const PoseStamped & pose_a, const PoseStamped & pose_b)
{
  return (rclcpp::Time(pose_b.header.stamp) - rclcpp::Time(pose_a.header.stamp)).seconds();
}

double calc_pose_velocity(const PoseStamped & pose_a, const PoseStamped & pose_b)
{
  const double dt = calc_time_diff(pose_a, pose_b);
  if (std::abs(dt) < std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  // Project the displacement onto the body longitudinal axis rather than taking its norm. The norm
  // is non-negative, so pose noise perpendicular to the travel direction inflates it by
  // sigma_perp^2 / (2 d) irrespective of sign, and the Kalman filter cannot average that away. The
  // projection is linear in the position error, so a zero-mean pose error stays zero-mean. It also
  // matches what VelocityReport measures (speed along the body x-axis) and keeps the sign, so
  // reverse motion is rejected by the min_speed gate instead of driving the scale factor negative.
  const auto axis_a = body_x_axis(pose_a.pose.orientation);
  const auto axis_b = body_x_axis(pose_b.pose.orientation);
  double ux = axis_a[0] + axis_b[0];
  double uy = axis_a[1] + axis_b[1];
  double uz = axis_a[2] + axis_b[2];
  const double u_norm = std::sqrt(ux * ux + uy * uy + uz * uz);
  if (u_norm < std::numeric_limits<double>::epsilon()) {
    return 0.0;  // degenerate: the two headings oppose each other
  }
  ux /= u_norm;
  uy /= u_norm;
  uz /= u_norm;

  // The vertical component is kept so that the result is the speed along the road surface on a
  // slope, which is what the wheel-derived velocity reports. Dropping it would make the scale
  // factor absorb an average cos(gradient) over the estimation route.
  const double dx = pose_b.pose.position.x - pose_a.pose.position.x;
  const double dy = pose_b.pose.position.y - pose_a.pose.position.y;
  const double dz = pose_b.pose.position.z - pose_a.pose.position.z;

  return (dx * ux + dy * uy + dz * uz) / dt;
}

std::optional<NearestImuSample> find_nearest_imu(
  const std::vector<Imu> & imus, const rclcpp::Time & target_time)
{
  if (imus.empty()) {
    return std::nullopt;
  }

  const Imu * nearest_imu = &imus.front();
  double min_stamp_diff =
    std::abs((rclcpp::Time(nearest_imu->header.stamp) - target_time).seconds());

  for (const auto & imu : imus) {
    const double stamp_diff = std::abs((rclcpp::Time(imu.header.stamp) - target_time).seconds());
    if (stamp_diff < min_stamp_diff) {
      min_stamp_diff = stamp_diff;
      nearest_imu = &imu;
    }
  }

  NearestImuSample sample;
  sample.angular_velocity_z = nearest_imu->angular_velocity.z;
  sample.stamp_diff = min_stamp_diff;
  return sample;
}

std::optional<NearestVelocityReportSample> find_nearest_velocity_report(
  const std::vector<VelocityReport> & velocity_reports, const rclcpp::Time & target_time)
{
  if (velocity_reports.empty()) {
    return std::nullopt;
  }

  const VelocityReport * nearest_velocity_report = &velocity_reports.front();
  double min_stamp_diff =
    std::abs((rclcpp::Time(nearest_velocity_report->header.stamp) - target_time).seconds());

  for (const auto & velocity_report : velocity_reports) {
    const double stamp_diff =
      std::abs((rclcpp::Time(velocity_report.header.stamp) - target_time).seconds());
    if (stamp_diff < min_stamp_diff) {
      min_stamp_diff = stamp_diff;
      nearest_velocity_report = &velocity_report;
    }
  }

  NearestVelocityReportSample sample;
  sample.longitudinal_velocity = nearest_velocity_report->longitudinal_velocity;
  sample.stamp_diff = min_stamp_diff;
  return sample;
}

std::optional<NearestEnvironmentIdSample> find_nearest_environment_id(
  const std::vector<Int32Stamped> & environment_ids, const rclcpp::Time & target_time)
{
  if (environment_ids.empty()) {
    return std::nullopt;
  }

  const Int32Stamped * nearest_environment_id = &environment_ids.front();
  double min_stamp_diff =
    std::abs((rclcpp::Time(nearest_environment_id->stamp) - target_time).seconds());

  for (const auto & environment_id : environment_ids) {
    const double stamp_diff =
      std::abs((rclcpp::Time(environment_id.stamp) - target_time).seconds());
    if (stamp_diff < min_stamp_diff) {
      min_stamp_diff = stamp_diff;
      nearest_environment_id = &environment_id;
    }
  }

  NearestEnvironmentIdSample sample;
  sample.environment_id = nearest_environment_id->data;
  sample.stamp_diff = min_stamp_diff;
  return sample;
}

std::optional<WorstNdtCorrectionSample> find_worst_ndt_correction(
  const std::vector<PoseStamped> & ndt_corrections, const rclcpp::Time & begin_time,
  const rclcpp::Time & end_time)
{
  if (ndt_corrections.empty()) {
    return std::nullopt;
  }

  const PoseStamped * nearest_to_begin = &ndt_corrections.front();
  const PoseStamped * nearest_to_end = &ndt_corrections.front();
  double min_begin_stamp_diff =
    std::abs((rclcpp::Time(nearest_to_begin->header.stamp) - begin_time).seconds());
  double min_end_stamp_diff =
    std::abs((rclcpp::Time(nearest_to_end->header.stamp) - end_time).seconds());

  double worst_longitudinal = 0.0;
  double worst_lateral = 0.0;

  for (const auto & ndt_correction : ndt_corrections) {
    const rclcpp::Time stamp(ndt_correction.header.stamp);

    const double begin_stamp_diff = std::abs((stamp - begin_time).seconds());
    if (begin_stamp_diff < min_begin_stamp_diff) {
      min_begin_stamp_diff = begin_stamp_diff;
      nearest_to_begin = &ndt_correction;
    }

    const double end_stamp_diff = std::abs((stamp - end_time).seconds());
    if (end_stamp_diff < min_end_stamp_diff) {
      min_end_stamp_diff = end_stamp_diff;
      nearest_to_end = &ndt_correction;
    }

    if (stamp < begin_time || end_time < stamp) {
      continue;
    }
    worst_longitudinal = std::max(worst_longitudinal, std::abs(ndt_correction.pose.position.x));
    worst_lateral = std::max(worst_lateral, std::abs(ndt_correction.pose.position.y));
  }

  // Both ends contribute to the pose velocity, so make sure they are always taken into account even
  // when their stamps fall marginally outside the window.
  for (const auto * endpoint : {nearest_to_begin, nearest_to_end}) {
    worst_longitudinal = std::max(worst_longitudinal, std::abs(endpoint->pose.position.x));
    worst_lateral = std::max(worst_lateral, std::abs(endpoint->pose.position.y));
  }

  WorstNdtCorrectionSample sample;
  sample.longitudinal = worst_longitudinal;
  sample.lateral = worst_lateral;
  sample.begin_stamp_diff = min_begin_stamp_diff;
  sample.end_stamp_diff = min_end_stamp_diff;
  return sample;
}

}  // namespace autoware::speed_scale_estimator

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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <rclcpp/time.hpp>

#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace autoware::speed_scale_estimator
{

using autoware_internal_debug_msgs::msg::Int32Stamped;
using autoware_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::Imu;

struct NearestImuSample
{
  double angular_velocity_z{};  //!< IMU z-axis angular velocity [rad/s]
  double stamp_diff{};          //!< Absolute time difference from target [s]
};

struct NearestVelocityReportSample
{
  double longitudinal_velocity{};  //!< Longitudinal velocity [m/s]
  double stamp_diff{};             //!< Absolute time difference from target [s]
};

struct NearestEnvironmentIdSample
{
  int32_t environment_id{};  //!< Environment id reported by environment_adaptor
  double stamp_diff{};       //!< Absolute time difference from target [s]
};

/**
 * @brief Worst NDT correction (initial pose -> converged pose) observed over a time window.
 *
 * The correction is expressed in the NDT initial (predicted) pose frame, so the longitudinal and
 * lateral components are directly the along-heading and cross-heading corrections applied by NDT.
 */
struct WorstNdtCorrectionSample
{
  double longitudinal{};      //!< Max |x| over the window [m]
  double lateral{};           //!< Max |y| over the window [m]
  double begin_stamp_diff{};  //!< Time difference to the sample nearest the window begin [s]
  double end_stamp_diff{};    //!< Time difference to the sample nearest the window end [s]
};

[[nodiscard]] double calc_time_diff(const PoseStamped & pose_a, const PoseStamped & pose_b);

[[nodiscard]] double calc_pose_velocity(const PoseStamped & pose_a, const PoseStamped & pose_b);

[[nodiscard]] std::optional<NearestImuSample> find_nearest_imu(
  const std::vector<Imu> & imus, const rclcpp::Time & target_time);

[[nodiscard]] std::optional<NearestVelocityReportSample> find_nearest_velocity_report(
  const std::vector<VelocityReport> & velocity_reports, const rclcpp::Time & target_time);

[[nodiscard]] std::optional<NearestEnvironmentIdSample> find_nearest_environment_id(
  const std::vector<Int32Stamped> & environment_ids, const rclcpp::Time & target_time);

/**
 * @brief Collect the worst NDT correction over [begin_time, end_time].
 *
 * The pose velocity is derived from both ends of the window, so a large correction anywhere in
 * between corrupts the observation. Samples strictly inside the window are folded together with the
 * samples nearest to each end, and the returned stamp differences let the caller reject windows
 * that the buffer does not actually cover.
 */
[[nodiscard]] std::optional<WorstNdtCorrectionSample> find_worst_ndt_correction(
  const std::vector<PoseStamped> & ndt_corrections, const rclcpp::Time & begin_time,
  const rclcpp::Time & end_time);

}  // namespace autoware::speed_scale_estimator

#endif  // UTILS_HPP_

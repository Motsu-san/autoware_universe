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

/**
 * @brief Wheel velocity time-averaged over a window, together with how well the buffer covers it.
 *
 * Mirrors WorstNdtCorrectionSample: the value is aggregated over the whole window, and the two
 * stamp differences let the caller reject windows the buffer does not actually cover. The
 * interpolated endpoint values are reported as well so that the caller can derive the longitudinal
 * acceleration for diagnostics without a second scan of the buffer.
 */
struct MeanVelocityReportSample
{
  double longitudinal_velocity{};        //!< Time-average over the window [m/s]
  double begin_longitudinal_velocity{};  //!< Interpolated value at the window begin [m/s]
  double end_longitudinal_velocity{};    //!< Interpolated value at the window end [m/s]
  double begin_stamp_diff{};             //!< Time difference to the sample nearest the begin [s]
  double end_stamp_diff{};               //!< Time difference to the sample nearest the end [s]
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

/**
 * @brief Longitudinal velocity implied by two consecutive poses.
 *
 * The displacement is projected onto the body longitudinal axis rather than reduced to its norm.
 * The norm is non-negative, so it turns pose noise perpendicular to the travel direction into a
 * systematic inflation of sigma_perp^2 / (2 d) that the Kalman filter cannot average away; the
 * projection is linear in the position error and therefore unbiased. It is also the quantity
 * VelocityReport reports, i.e. the speed along the body x-axis.
 *
 * The result is signed: reverse motion yields a negative value, which the min_speed constraint
 * rejects. The vertical component is included, so on a slope this is the speed along the road
 * surface, matching wheel-derived velocity.
 */
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

/**
 * @brief Wheel velocity averaged over [begin_time, end_time].
 *
 * The pose velocity this is compared against is a displacement divided by the interval, i.e. the
 * MEAN velocity over [t_prev, t_curr], not the instantaneous velocity at either end. Pairing it
 * with the single wheel sample nearest t_curr compares two quantities half an interval apart, so
 * under a longitudinal acceleration a the ratio carries a bias of -a * dt / (2 * v_wheel): 0.47 %
 * at a = 0.5 m/s^2, dt = 0.3 s, v = 16 m/s, i.e. larger than the 0.1-0.3 % effects the scale
 * factor is used to resolve. The bias is systematic rather than zero-mean, because a vehicle on a
 * fixed route accelerates in the same places on every pass, so the Kalman filter cannot average it
 * away and it masquerades as a place-dependent scale factor. Averaging the wheel velocity over the
 * same interval makes z = x * v_wheel an identity for any constant x and any acceleration profile,
 * which is why no acceleration gate is needed on top of it.
 *
 * The average is the trapezoid rule over the samples strictly inside the window plus a linear
 * interpolation at each end. Outside the sampled range the interpolation is clamped rather than
 * extrapolated, so a stale buffer cannot invent motion that was never measured; the returned stamp
 * differences let the caller reject such windows, the same contract as find_worst_ndt_correction().
 *
 * A window of non-positive width has no interval to average over, so the value interpolated at
 * end_time is returned instead. That is the limit of the average as the window shrinks, and unlike
 * a NaN it cannot propagate into the Kalman gain and poison the recursive estimate; the pose
 * velocity is zero or negative for the same input, so the min_speed constraint rejects it anyway.
 *
 * @pre `velocity_reports` is sorted ascending by stamp, which the rolling buffer guarantees by
 *   dropping out-of-order inserts. Samples that go backwards are skipped rather than integrated
 *   with a negative width, but the result for unsorted input is not meaningful.
 */
[[nodiscard]] std::optional<MeanVelocityReportSample> calc_mean_velocity_report(
  const std::vector<VelocityReport> & velocity_reports, const rclcpp::Time & begin_time,
  const rclcpp::Time & end_time);

}  // namespace autoware::speed_scale_estimator

#endif  // UTILS_HPP_

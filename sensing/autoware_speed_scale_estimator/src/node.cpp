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

#include "node.hpp"

#include <cstdint>
#include <functional>
#include <vector>

namespace autoware::speed_scale_estimator
{
namespace
{

SpeedScaleEstimatorParameters load_parameters(rclcpp::Node * node)
{
  SpeedScaleEstimatorParameters parameters;
  parameters.update_interval = node->declare_parameter<double>("update_interval");
  parameters.max_pose_lag = node->declare_parameter<double>("max_pose_lag");
  parameters.max_stamp_lag = node->declare_parameter<double>("max_stamp_lag");
  parameters.sensor_buffer_duration = node->declare_parameter<double>("sensor_buffer_duration");
  parameters.initial_speed_scale_factor =
    node->declare_parameter<double>("initial_speed_scale_factor");
  parameters.initial_speed_scale_factor_covariance =
    node->declare_parameter<double>("initial_speed_scale_factor_covariance");
  parameters.process_noise_covariance = node->declare_parameter<double>("process_noise_covariance");
  parameters.measurement_noise_covariance =
    node->declare_parameter<double>("measurement_noise_covariance");
  parameters.max_angular_velocity = node->declare_parameter<double>("max_angular_velocity");
  parameters.max_speed = node->declare_parameter<double>("max_speed");
  parameters.min_speed = node->declare_parameter<double>("min_speed");
  parameters.enable_ndt_stability_check =
    node->declare_parameter<bool>("enable_ndt_stability_check");
  parameters.max_ndt_longitudinal_correction =
    node->declare_parameter<double>("max_ndt_longitudinal_correction");
  parameters.max_ndt_lateral_correction =
    node->declare_parameter<double>("max_ndt_lateral_correction");
  parameters.enable_degenerate_environment_check =
    node->declare_parameter<bool>("enable_degenerate_environment_check");
  parameters.allowed_environment_ids =
    node->declare_parameter<std::vector<int64_t>>("allowed_environment_ids");
  return parameters;
}

}  // namespace

SpeedScaleEstimatorNode::SpeedScaleEstimatorNode(const rclcpp::NodeOptions & node_options)
: Node("speed_scale_estimator", node_options),
  parameters_(load_parameters(this)),
  processor_(parameters_)
{
  pub_estimated_speed_scale_factor_ = create_publisher<Float32Stamped>("~/output/scale_factor", 1);
  pub_debug_info_ = create_publisher<StringStamped>("~/output/debug_info", 1);

  sub_pose_ =
    PollingSubscriber<PoseStamped, All>::create_subscription(this, "~/input/pose", rclcpp::QoS{10});
  sub_velocity_report_ = PollingSubscriber<VelocityReport, All>::create_subscription(
    this, "~/input/velocity_report", rclcpp::QoS{10});
  sub_imu_ = PollingSubscriber<Imu, All>::create_subscription(this, "~/input/imu", rclcpp::QoS{10});

  if (parameters_.enable_ndt_stability_check) {
    sub_ndt_correction_ = PollingSubscriber<PoseStamped, All>::create_subscription(
      this, "~/input/ndt_initial_to_result_relative_pose", rclcpp::QoS{10});
  }
  if (parameters_.enable_degenerate_environment_check) {
    sub_environment_id_ = PollingSubscriber<Int32Stamped, All>::create_subscription(
      this, "~/input/environment_id", rclcpp::QoS{10});
  }

  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<double>(processor_.get_update_interval_sec()),
    std::bind(&SpeedScaleEstimatorNode::on_timer, this));
}

void SpeedScaleEstimatorNode::on_timer()
{
  const auto ndt_correction_ptrs = sub_ndt_correction_ ? sub_ndt_correction_->take_data()
                                                       : std::vector<PoseStamped::ConstSharedPtr>{};
  const auto environment_id_ptrs = sub_environment_id_
                                     ? sub_environment_id_->take_data()
                                     : std::vector<Int32Stamped::ConstSharedPtr>{};

  const auto result = processor_.process(
    sub_pose_->take_data(), sub_imu_->take_data(), sub_velocity_report_->take_data(),
    ndt_correction_ptrs, environment_id_ptrs);

  const auto now = get_clock()->now();
  pub_debug_info_->publish(processor_.make_debug_info(result, now));

  if (result.updated) {
    pub_estimated_speed_scale_factor_->publish(
      processor_.make_scale_factor_msg(result.estimation_result.value(), now));
  }
}

}  // namespace autoware::speed_scale_estimator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::speed_scale_estimator::SpeedScaleEstimatorNode)

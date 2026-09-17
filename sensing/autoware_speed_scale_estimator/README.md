# autoware_speed_scale_estimator

## Overview

This package estimates the vehicle speed scale factor by comparing pose-derived velocity from consecutive localization poses with wheel velocity reported by the vehicle. The reference velocity is computed from pose position differences (e.g. NDT scan matcher output), so the estimate reflects consistency between localization motion and wheel speed.

The estimated scale factor can be used to monitor and tune the speed scale in `autoware_vehicle_velocity_converter`. The node runs periodically at `update_interval` and updates the estimate only when all operational constraints are satisfied.

## Package Architecture

The package is split into a core library and a ROS adapter layer. The core implements estimation without ROS topic subscription or publication; it uses `rclcpp::Time` only for timestamp handling. Core logic can be unit-tested without spinning up a ROS graph.

| Layer | Library                              | Responsibility                                                                                           |
| ----- | ------------------------------------ | -------------------------------------------------------------------------------------------------------- |
| Core  | `autoware_speed_scale_estimator`     | Utils, Kalman filter estimation (`SpeedScaleEstimator`); no ROS graph I/O                                |
| ROS   | `autoware_speed_scale_estimator_ros` | Message handling, debug formatting, node I/O (`SpeedScaleEstimatorProcessor`, `SpeedScaleEstimatorNode`) |

## Block Diagram

### Node architecture

ROS node interfaces and publish timing.

![node architecture](docs/block_diagram_node.drawio.svg)

### Estimation algorithm

Internal processing flow inside `SpeedScaleEstimator`.

![estimation algorithm](docs/block_diagram_algorithm.drawio.svg)

## Algorithm

The speed scale estimation follows these steps:

### 1. Data Collection

- Collects pose (`PoseStamped`), IMU, and vehicle velocity reports (`VelocityReport`) received since the previous timer callback
- Optionally collects the NDT initial-to-result relative pose (`PoseStamped`) and the environment id (`Int32Stamped`) used by the localization quality constraints, when the corresponding constraint is enabled
- Keeps IMU, velocity report, and the optional localization quality messages in rolling buffers of length `sensor_buffer_duration` for timestamp synchronization
- Keeps the previous pose internally to compute pose velocity between consecutive updates

### 2. Pose Velocity Calculation

Computes pose velocity by projecting the displacement between the previous and current poses onto the body longitudinal axis:

$$
v_{pose} = \frac{\Delta \mathbf{p} \cdot \hat{\mathbf{u}}}{\Delta t}, \quad \hat{\mathbf{u}} = \frac{\hat{\mathbf{u}}_{prev} + \hat{\mathbf{u}}_{curr}}{\| \hat{\mathbf{u}}_{prev} + \hat{\mathbf{u}}_{curr} \|}
$$

where $\hat{\mathbf{u}}_i$ is the body x-axis of pose $i$. The two are averaged so that the axis stays meaningful if `max_angular_velocity` is ever relaxed; under the default gate the two differ by only ~3 mrad.

The projection is used instead of the displacement norm $\| \Delta \mathbf{p} \|$ for three reasons:

- **The norm is biased.** It is non-negative, so pose noise perpendicular to the travel direction inflates it regardless of sign: $E[\| \Delta \mathbf{p} \|] \approx d + \sigma_\perp^2 / (2d)$. That is a systematic error, so the Kalman filter cannot average it away, and its relative size $\sigma_\perp^2 / (2d^2)$ grows as the baseline shortens — worst exactly at `min_speed`. The projection is linear in the position error, so a zero-mean pose error stays zero-mean, and the residual error becomes $\varepsilon^2/2$ in the orientation error $\varepsilon$, which does not depend on $d$.
- **It matches the observation model.** `VelocityReport.longitudinal_velocity` is the speed along the body x-axis, so projecting onto the same axis makes $z = x \cdot v_{wheel}$ exact rather than approximate up to the side-slip angle.
- **It keeps the sign.** The wheel velocity report is signed. A non-negative $v_{pose}$ would pair a positive observation with a negative $H$ during reverse motion and drive the scale factor negative; a signed $v_{pose}$ is rejected by the `min_speed` constraint instead.

All three components of $\Delta \mathbf{p}$ including the vertical one participate, so on a slope this is the speed along the road surface — the same quantity the wheels measure. Reducing this to a horizontal 2D displacement would make the scale factor absorb an average $\cos(\text{gradient})$ over the estimation route (0.12 % on a 5 % grade, 0.50 % on a 10 % grade) and make the estimate route-dependent.

Being a displacement quotient, $v_{pose}$ is the *mean* velocity over $[t_{prev}, t_{curr}]$ rather than the instantaneous velocity at either end. The wheel velocity it is paired with is averaged over the same interval for that reason; see step 3.

If the time difference between the two poses exceeds `max_pose_lag`, the previous pose is reset and estimation is skipped.

### 3. Timestamp Synchronization

For the current pose timestamp, the estimator selects from the rolling buffers:

- the nearest IMU sample for angular velocity constraint checking
- the nearest environment id, when the degenerate environment constraint is enabled

Estimation is skipped when either of these timestamp differences exceeds `max_stamp_lag`.

Two quantities are deliberately **not** single-sample lookups, for the same reason: $v_{pose}$ is derived from both ends of the pose interval, so anything paired with it must refer to the same interval $[t_{prev}, t_{curr}]$.

- **Wheel velocity.** $v_{wheel}$ is the time-average of `VelocityReport.longitudinal_velocity` over the interval, computed with the trapezoid rule over the samples inside it plus a linear interpolation at each end. A single sample at $t_{curr}$ would be $v(t_{curr})$ while $v_{pose}$ is $\bar{v} \approx v(t_{mid})$, half an interval earlier, so under a longitudinal acceleration $a$ the ratio carries a bias of $-a \Delta t / (2 v_{wheel})$ — 0.47 % at $a = 0.5\ \mathrm{m/s^2}$, $\Delta t = 0.3\ \mathrm{s}$, $v = 16\ \mathrm{m/s}$, i.e. larger than the 0.1–0.3 % effects this estimator is used to resolve. The bias is systematic rather than zero-mean, because a vehicle on a fixed route accelerates in the same places on every pass; the Kalman filter therefore cannot average it away and it masquerades as a place-dependent scale factor. Averaging over the same interval cancels it for any acceleration profile, which is why no acceleration gate is needed on top of it.
- **NDT correction.** The estimator takes the worst correction over $[t_{prev}, t_{curr}]$, because a large correction anywhere in between corrupts the observation.

Both additionally require a sample within `max_stamp_lag` of *each* end, so that the interval is actually covered by the buffer. Outside the sampled range the wheel velocity is clamped rather than extrapolated, so a stale buffer cannot invent motion that was never measured. This makes `sensor_buffer_duration` > `max_pose_lag` + `max_stamp_lag` a requirement for the interval to be coverable at all (1.0 > 0.5 + 0.2 by default).

### 4. Constraint Validation

Estimation is performed only when all of the following constraints are satisfied:

- **Angular velocity constraint (IMU)** — $|\omega_{imu}| \leq \omega_{max}$

  Default $\omega_{max} \approx 0.6\ \mathrm{deg/s}$ ($0.0105\ \mathrm{rad/s}$) rejects obvious curves and turns. This threshold is set near the practical IMU bias/noise floor rather than fine straight-line discrimination.

- **Speed constraints (pose velocity)** — $v_{min} \leq v_{pose} \leq v_{max}$

  Default $v_{min} = 6.0\ \mathrm{m/s}$ rejects low-speed samples where pose-differentiation SNR is poor: at short $\Delta t$, the displacement $\Delta \mathbf{p}$ becomes comparable to localization pose noise (e.g. NDT jitter), so relative error in $v_{pose}$ grows and corrupts the scale-factor observation $z = v_{pose}$. Note that this is about variance, not bias — the projection in step 2 removes the baseline-dependent bias that the displacement norm would carry. Wheel speed reports also tend to have larger relative uncertainty near standstill or creep. In practice, sub-6 m/s segments are often urban and involve frequent acceleration/deceleration, which further degrades consistency between pose-derived and wheel-reported velocity. Default $v_{max} = 17.0\ \mathrm{m/s}$ filters out extreme speeds. Together with $\omega_{max}$, estimation runs mainly on moderate-speed, near-straight segments (~22 km/h and above).

  Estimation is intentionally skipped in urban/low-speed segments. Lower `min_speed` if broader coverage is needed, accepting higher estimate variance.

- **Wheel velocity validity** — $|v_{wheel}| > 10^{-6}$ (applied to the interval average defined in step 3, i.e. to the value that actually becomes the observation matrix)

- **NDT scan matching stability** (`enable_ndt_stability_check`, enabled by default)

  `ndt_scan_matcher` publishes `initial_to_result_relative_pose`, which is its converged pose expressed in the frame of the initial guess it started from. Its `position.x` is therefore the correction NDT applied _along_ the predicted heading and `position.y` the correction _across_ it. Estimation requires $|x| \leq$ `max_ndt_longitudinal_correction` and $|y| \leq$ `max_ndt_lateral_correction` over the whole pose interval.

  The longitudinal axis is the one that degrades in degenerate environments, and it is also the axis that directly corrupts $v_{pose}$, so a large correction there is exactly the condition to reject. The correspondence is exact: the initial guess is a predicted `base_link` pose, so the frame the correction is expressed in shares its x-axis with the $\hat{\mathbf{u}}$ that step 2 projects onto. Defaults of 0.15 m match the order of the empirical NDT position standard deviation configured in `ndt_scan_matcher` (`output_pose_covariance[0]` = 0.0225, i.e. $\sigma = 0.15\ \mathrm{m}$). The `initial_to_result_distance_tolerance_m` diagnostic threshold (3.0 m by default) is deliberately not reused: it is sized to detect NDT breakdown, not the sub-decimeter jitter that matters here.

  The vertical component `position.z` is deliberately not gated. It reaches $v_{pose}$ only through $\hat{u}_z = -\sin(\text{pitch})$, so on ordinary grades it is attenuated by more than an order of magnitude, and what remains is zero-mean rather than a bias — the projection in step 2 already removes the one term through which a vertical error would otherwise accumulate systematically.

  This constraint is used instead of the `pose_instability_detector` diagnostic. That detector compares the EKF pose against gyro-plus-wheel dead reckoning rather than looking at NDT directly, and its thresholds include `heading_velocity_scale_factor_tolerance` (3 % by default) — so a wheel speed scale error above 3 % would itself trip the detector and gate out precisely the samples needed to estimate that error. `initial_to_result_relative_pose` shares the same initial-pose dependency in principle, but it only accumulates one scan (~0.1 s) of prediction error, roughly 0.05 m at 16 m/s with a 3 % scale error, which stays well inside the 0.15 m threshold.

- **Degenerate environment** (`enable_degenerate_environment_check`, disabled by default)

  `environment_adaptor` publishes `~/debug/environment_id`, and estimation is allowed only when it is contained in `allowed_environment_ids`. With the default `environment_adaptor` configuration, 0 is a normal environment, 1 a uniform road, and 2 a feature-poor road. In a degenerate area NDT slips longitudinally, so $\Delta \mathbf{p} \cdot \hat{\mathbf{u}}$ no longer tracks the true traveled distance.

  Two caveats explain the default:

  - The judgement is purely a 2D point-in-polygon test against `degenerate_area` polygons annotated in the lanelet2 map — there is no online degeneracy detection — so the constraint is only as good as the map annotation.
  - `environment_adaptor` is not launched by default (`use_autoware_environment_adaptor` is false), so the topic is normally absent.

  When the constraint is enabled and no sufficiently recent environment id is available, estimation is skipped rather than assumed safe. The subscription is only created when the constraint is enabled, so the default configuration adds no topic dependency.

  This also matches how `environment_adaptor` is meant to be used: it applies a per-area `longitudinal_scale_factor` from the map inside degenerate areas, so the global scale factor is best estimated outside them.

### 5. Scale Factor Estimation (Kalman Filter)

The scale factor is modeled as a scalar state $x$ with the observation:

$$
z = v_{pose} = x \cdot v_{wheel}
$$

where both sides are means over the same interval:

$$
v_{pose} = \frac{1}{\Delta t}\int_{t_{prev}}^{t_{curr}} v(\tau)\\,d\tau, \qquad
v_{wheel} = \frac{1}{\Delta t}\int_{t_{prev}}^{t_{curr}} v_{wheel}(\tau)\\,d\tau
$$

Because the integral is linear, this is an exact identity whenever $x$ is constant over the interval — it does not require $\dot{v} = 0$, which is what pairing $v_{pose}$ with a point sample would implicitly assume.

Kalman filter update:

**Prediction:**

$$
x_{pred} = x,\quad P_{pred} = P + Q
$$

**Update:**

$$
H = v_{wheel},\quad y = z - H x_{pred}
$$

$$
S = H^2 P_{pred} + R,\quad K = \frac{P_{pred} H}{S}
$$

$$
x = x_{pred} + K y,\quad P = (1 - K H) P_{pred}
$$

## Inputs / Outputs

### Input Topics

| Name                                          | Type                                              | Description                                                                                                                                       |
| --------------------------------------------- | ------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `~/input/pose`                                | `geometry_msgs::msg::PoseStamped`                 | Localization pose (e.g. NDT scan matcher)                                                                                                         |
| `~/input/velocity_report`                     | `autoware_vehicle_msgs::msg::VelocityReport`      | Vehicle velocity report                                                                                                                           |
| `~/input/imu`                                 | `sensor_msgs::msg::Imu`                           | IMU sensor data                                                                                                                                   |
| `~/input/ndt_initial_to_result_relative_pose` | `geometry_msgs::msg::PoseStamped`                 | NDT correction from the initial guess to the converged pose, in the initial pose frame. Subscribed only when `enable_ndt_stability_check` is true |
| `~/input/environment_id`                      | `autoware_internal_debug_msgs::msg::Int32Stamped` | Environment id from `environment_adaptor`. Subscribed only when `enable_degenerate_environment_check` is true                                     |

### Output Topics

| Name                    | Type                                                | Description                                                        |
| ----------------------- | --------------------------------------------------- | ------------------------------------------------------------------ |
| `~/output/scale_factor` | `autoware_internal_debug_msgs::msg::Float32Stamped` | Estimated speed scale factor (published on successful update only) |
| `~/output/debug_info`   | `autoware_internal_debug_msgs::msg::StringStamped`  | Debug information (published every timer callback)                 |

## Parameters

{{ json_to_markdown("sensing/autoware_speed_scale_estimator/schema/speed_scale_estimator.schema.json") }}

### Tuning

Default constraint values are a practical starting point, not vehicle-specific optima. If estimation accuracy is insufficient, tune `max_pose_lag`, `max_stamp_lag`, `sensor_buffer_duration`, `max_angular_velocity`, `min_speed`, `max_speed`, and Kalman noise parameters per vehicle and operation. For urban or general-road monitoring, consider lowering `min_speed` and increasing `measurement_noise_covariance` to tolerate noisier pose-differentiation observations. IMU bias cancellation or other preprocessing may allow a stricter angular velocity threshold; without it, thresholds near the IMU noise floor are expected.

Note that `max_stamp_lag` now also bounds how far short of *either* end of the pose interval the velocity report and NDT correction buffers may fall, so `sensor_buffer_duration` must stay above `max_pose_lag` + `max_stamp_lag`. Raising `max_pose_lag` toward `sensor_buffer_duration` will start producing `VelocityReportTimestampMismatch` rejections.

`max_ndt_longitudinal_correction` and `max_ndt_lateral_correction` should be set from the observed distribution of `initial_to_result_relative_pose` (or `initial_to_result_distance`) on the target route. Tightening them improves the quality of each accepted sample but reduces how often estimation runs; if the accepted sample rate drops sharply, relax them rather than lowering `min_speed`. Set `enable_ndt_stability_check` to false to restore the pre-existing behavior of not looking at localization quality at all.

`enable_degenerate_environment_check` requires `use_autoware_environment_adaptor` to be true and the lanelet2 map to carry `degenerate_area` polygons. Keep it disabled until both hold, because with the constraint enabled and the topic absent the estimator intentionally never updates.

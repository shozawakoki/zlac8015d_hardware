#ifndef ZLAC8015D_MIMO_CONTROL_HPP_
#define ZLAC8015D_MIMO_CONTROL_HPP_

// MIMO current-command controllers for a differential-drive robot.
// Physical wheel order is [left, right]; body order is [v, yaw_rate].
// Positive yaw is left/CCW: yaw_rate = r / track * (right - left).
// No ROS dependency: can be unit tested using a standard C++17 compiler.

#include <array>
#include <algorithm>
#include <cstddef>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace zlac8015d_hardware
{
namespace mimo
{

using Vec2 = std::array<double, 2>;
using Mat2 = std::array<Vec2, 2>;

// BREAKAWAY is intended to overcome static friction, not to hold an open-loop
// torque against a wheel that is already rotating. In MIMO torque modes,
// permit TRACK entry after sustained measured motion in EITHER direction.
// This lets closed-loop feedback command braking when motion and reference
// have opposite signs. Legacy per-wheel modes keep direction-specific entry.
// The caller must require consecutive qualified samples to reject noise.
inline bool moving_for_track(
  const double reference_rad_s, const double measured_rad_s,
  const double threshold_rad_s, const bool mimo_torque_mode)
{
  if (!std::isfinite(reference_rad_s) ||
      !std::isfinite(measured_rad_s) ||
      !std::isfinite(threshold_rad_s) || threshold_rad_s <= 0.0 ||
      std::abs(reference_rad_s) <= 1.0e-6)
  {
    return false;
  }

  const double signed_speed =
    std::copysign(1.0, reference_rad_s) * measured_rad_s;
  return mimo_torque_mode ?
    std::abs(measured_rad_s) >= threshold_rad_s :
    signed_speed >= threshold_rad_s;
}

// In a coordinated turn, a wheel already moving should wait at its
// existing breakaway current, not jump to the maximum merely because
// its partner has not begun moving. Keep non-MIMO legacy behavior.
inline double coordinated_ready_target(
  const double existing_target_a, const double legacy_wait_a,
  const bool mimo_torque_mode)
{
  return mimo_torque_mode ?
    existing_target_a : std::max(existing_target_a, legacy_wait_a);
}

// MIMO fault detection must use body yaw tracking as well as individual wheel
// measurements. Feedback alone cannot satisfy [v, yaw] if a wheel is stuck;
// instead, latch a coordinated stop when a large yaw residual persists while
// one commanded wheel is stationary and its partner is moving.
// Index: 0=left, 1=right; no CAN or ROS dependencies, unit-testable.
struct CoordinatedStallGuard
{
  double mismatch_duration_s{0.0};
  bool latched{false};
  int stalled_wheel{-1};

  void reset()
  {
    mismatch_duration_s = 0.0;
    latched = false;
    stalled_wheel = -1;
  }

  bool update(
    const Vec2 & wheel_reference, const Vec2 & wheel_measured,
    const double linear_reference, const double linear_measured,
    const double yaw_reference, const double yaw_measured,
    const double dt)
  {
    if (latched) {return true;}
    if (!std::isfinite(wheel_reference[0]) ||
        !std::isfinite(wheel_reference[1]) ||
        !std::isfinite(wheel_measured[0]) ||
        !std::isfinite(wheel_measured[1]) ||
        !std::isfinite(linear_reference) || !std::isfinite(linear_measured) ||
        !std::isfinite(yaw_reference) || !std::isfinite(yaw_measured) ||
        !std::isfinite(dt) || dt <= 0.0) {
      mismatch_duration_s = 0.0;
      return false;
    }

    constexpr double kMinRequestedWheelSpeed = 1.0; // rad/s
    constexpr double kStuckSpeed = 0.20; // rad/s
    constexpr double kOtherWheelMoving = 1.0; // rad/s
    constexpr double kExcessYaw = 0.35; // rad/s
    constexpr double kLinearTrackingError = 0.08; // m/s
    constexpr double kConfirmDuration = 0.15; // s

    const bool both_requested =
      std::abs(wheel_reference[0]) >= kMinRequestedWheelSpeed &&
      std::abs(wheel_reference[1]) >= kMinRequestedWheelSpeed;
    const bool yaw_mismatch =
      std::abs(yaw_measured - yaw_reference) >= kExcessYaw;
    const bool linear_mismatch =
      std::abs(linear_measured - linear_reference) >= kLinearTrackingError;
    const bool left_stuck =
      std::abs(wheel_measured[0]) <= kStuckSpeed &&
      std::abs(wheel_measured[1]) >= kOtherWheelMoving;
    const bool right_stuck =
      std::abs(wheel_measured[1]) <= kStuckSpeed &&
      std::abs(wheel_measured[0]) >= kOtherWheelMoving;
    if (both_requested && yaw_mismatch && linear_mismatch &&
        (left_stuck || right_stuck)) {
      mismatch_duration_s += dt;
      if (mismatch_duration_s >= kConfirmDuration) {
        latched = true;
        stalled_wheel = left_stuck ? 0 : 1;
      }
    } else {
      mismatch_duration_s = 0.0;
    }
    return latched;
  }
};

// Safety-only deceleration after asymmetric stall. Force the sign opposite
// to measured motion; never keep applying drive torque in the wrong
// direction while a wheel is constrained. Normal LADRC is not modified.
inline double coupled_stall_brake_current(
  const double measured_speed, const double previous_current,
  const double dt, const double rate_limit_a_per_s)
{
  if (!std::isfinite(measured_speed) ||
      !std::isfinite(previous_current) || !std::isfinite(dt) ||
      !std::isfinite(rate_limit_a_per_s) ||
      dt <= 0.0 || rate_limit_a_per_s <= 0.0)
  {
    return 0.0;
  }
  if (std::abs(measured_speed) <= 0.10) {return 0.0;}
  const double target =
    -std::copysign(std::min(1.0, 0.25 * std::abs(measured_speed)),
                   measured_speed);
  const double max_step = rate_limit_a_per_s * dt;
  const double next = std::max(previous_current - max_step,
                     std::min(previous_current + max_step, target));
  // Cut a positive driving current immediately instead of prolonging it
  // during the initial sign change of an emergency brake.
  if (next * measured_speed > 0.0) {return 0.0;}
  return std::max(-1.0, std::min(1.0, next));
}

inline Vec2 multiply(const Mat2 & m, const Vec2 & v)
{
  return {m[0][0] * v[0] + m[0][1] * v[1],
          m[1][0] * v[0] + m[1][1] * v[1]};
}

inline Mat2 inverse(const Mat2 & a)
{
  const double det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
  if (!std::isfinite(det) || std::abs(det) < 1.0e-12) {
    throw std::invalid_argument("Singular/non-finite MIMO input matrix");
  }
  return {{{a[1][1] / det, -a[0][1] / det},
           {-a[1][0] / det, a[0][0] / det}}};
}

struct DifferentialDriveModel
{
  Mat2 T{};       // wheel angular speeds [L,R] -> body [v,omega]
  Mat2 T_inv{};
  Mat2 B{};       // commanded wheel currents [L,R] -> body acceleration
  Mat2 B_inv{};
  double nominal_b{18.0};

  DifferentialDriveModel(
    const double wheel_radius, const double wheel_separation,
    const double b_left, const double b_right)
  {
    if (!std::isfinite(wheel_radius) || wheel_radius <= 0.0 ||
        !std::isfinite(wheel_separation) || wheel_separation <= 0.0 ||
        !std::isfinite(b_left) || b_left <= 0.0 ||
        !std::isfinite(b_right) || b_right <= 0.0)
    {
      throw std::invalid_argument("Invalid MIMO wheel geometry or b0");
    }
    const double half_r = wheel_radius * 0.5;
    const double yaw_scale = wheel_radius / wheel_separation;
    T = {{{half_r, half_r}, {-yaw_scale, yaw_scale}}};
    T_inv = inverse(T);
    B = {{{T[0][0] * b_left, T[0][1] * b_right},
          {T[1][0] * b_left, T[1][1] * b_right}}};
    B_inv = inverse(B);
    nominal_b = 0.5 * (b_left + b_right);
  }

  Vec2 wheel_to_body(const Vec2 & wheels) const {return multiply(T, wheels);}
  Vec2 body_to_wheel(const Vec2 & body) const {return multiply(T_inv, body);}
  Vec2 acceleration_to_current(const Vec2 & body_acceleration) const
  {
    return multiply(B_inv, body_acceleration);
  }
};

// First-order MIMO-LADRC with the total disturbance as a second vector state.
// Euler LESO: z1_dot = z2 + B*u_previous + 2*Wo*(y-z1)
//             z2_dot = Wo^2*(y-z1).
// The same ROS-positive current that was actually sent must be fed to the ESO.
class Ladrc
{
public:
  Vec2 z1{{0.0, 0.0}};
  Vec2 z2{{0.0, 0.0}};
  bool initialized{false};

  void reset() { z1 = {{0.0, 0.0}}; z2 = {{0.0, 0.0}}; initialized = false; }
  void reset_to_measurement(const Vec2 & measured)
  {
    z1 = measured;
    z2 = {{0.0, 0.0}};
    initialized = true;
  }

  Vec2 step(
    const DifferentialDriveModel & model, const Vec2 & reference,
    const Vec2 & measured, const Vec2 & previous_current, const double dt,
    const Vec2 & wc, const Vec2 & wo, const Vec2 & acceleration_feedforward = {{0.0, 0.0}})
  {
    if (!initialized) {reset_to_measurement(measured);}
    const Vec2 known_acceleration = multiply(model.B, previous_current);
    for (std::size_t i = 0; i < 2; ++i) {
      const double error = measured[i] - z1[i];
      const double z1_dot = z2[i] + known_acceleration[i] + 2.0 * wo[i] * error;
      const double z2_dot = wo[i] * wo[i] * error;
      z1[i] += dt * z1_dot;
      z2[i] += dt * z2_dot;
    }
    Vec2 desired_acceleration{};
    for (std::size_t i = 0; i < 2; ++i) {
      desired_acceleration[i] =
        acceleration_feedforward[i] + wc[i] * (reference[i] - z1[i]) - z2[i];
    }
    return model.acceleration_to_current(desired_acceleration);
  }
};

// Body-coordinate PID. Gains are seeded from the wheel PID gains:
// acceleration-equivalent gains = mean(b0_left,b0_right)*original_pid_gain.
// With equal gains on v/yaw and equal wheel b0 this reproduces the wheel PID
// law after the T^-1 transform (except state initialization/actuator saturation).
// D acts on measured body velocity to avoid reference derivative kick.
class Pid
{
public:
  Vec2 integral_error{{0.0, 0.0}};
  Vec2 filtered_derivative{{0.0, 0.0}};
  Vec2 previous_measured{{0.0, 0.0}};
  std::chrono::steady_clock::time_point previous_sample_time{};
  // Diagnostics: nonzero only on a cycle that accepted a fresh speed sample.
  double derivative_sample_dt_s{0.0};
  bool derivative_updated{false};
  bool initialized{false};

  void reset()
  {
    integral_error = {{0.0, 0.0}};
    filtered_derivative = {{0.0, 0.0}};
    previous_measured = {{0.0, 0.0}};
    previous_sample_time = {};
    derivative_sample_dt_s = 0.0;
    derivative_updated = false;
    initialized = false;
  }

  Vec2 step(
    const DifferentialDriveModel & model, const Vec2 & reference,
    const Vec2 & measured, const double dt, const Vec2 & kp,
    const Vec2 & ki, const Vec2 & kd, const double derivative_tau,
    const double current_limit, const Vec2 & acceleration_feedforward = {{0.0, 0.0}},
    const bool enable_integral = true,
    const std::chrono::steady_clock::time_point velocity_sample_time = {})
  {
    derivative_sample_dt_s = 0.0;
    derivative_updated = false;
    // P and I continue to run at the controller rate (nominally 200 Hz).
    // D changes only when a *new TPDO0 velocity measurement* was received.
    // velocity_sample_time is updated by read() on every valid TPDO0 frame
    // and stays unchanged during controller cycles without a new frame.
    if (!initialized) {
      previous_measured = measured;
      previous_sample_time = velocity_sample_time;
      initialized = true;
    }

    if (velocity_sample_time != previous_sample_time &&
        velocity_sample_time != std::chrono::steady_clock::time_point{})
    {
      const double sample_dt = std::chrono::duration<double>(
        velocity_sample_time - previous_sample_time).count();

      // Reject non-monotonic and near-simultaneous read timestamps to avoid
      // extreme numerical derivatives from a buffered/bursty CAN stream.
      // Do not advance the derivative baseline for these rejected frames,
      // so a subsequent valid sample measures the accumulated interval.
      constexpr double MIN_SAMPLE_DT_S = 0.001;
      constexpr double MAX_SAMPLE_DT_S = 0.100;
      if (std::isfinite(sample_dt) &&
          sample_dt >= MIN_SAMPLE_DT_S && sample_dt <= MAX_SAMPLE_DT_S)
      {
        const double alpha = sample_dt / (derivative_tau + sample_dt);
        for (std::size_t i = 0; i < 2; ++i) {
          const double derivative =
            (measured[i] - previous_measured[i]) / sample_dt;
          filtered_derivative[i] += alpha *
            (derivative - filtered_derivative[i]);
        }
        previous_measured = measured;
        previous_sample_time = velocity_sample_time;
        derivative_sample_dt_s = sample_dt;
        derivative_updated = true;
      } else if (!std::isfinite(sample_dt) || sample_dt < 0.0 ||
                 sample_dt > MAX_SAMPLE_DT_S)
      {
        // Following a long gap or a clock discontinuity, do not form
        // a derivative across the gap; restart from the new sample.
        filtered_derivative = {{0.0, 0.0}};
        previous_measured = measured;
        previous_sample_time = velocity_sample_time;
      }
    }

    Vec2 error{};
    for (std::size_t i = 0; i < 2; ++i) {
      error[i] = reference[i] - measured[i];
    }
    const auto calculate = [&](const Vec2 & integ) -> Vec2 {
      Vec2 virtual_acceleration{};
      for (std::size_t i = 0; i < 2; ++i) {
        virtual_acceleration[i] = model.nominal_b *
          (kp[i] * error[i] + ki[i] * integ[i] - kd[i] * filtered_derivative[i]) +
          acceleration_feedforward[i];
      }
      return model.acceleration_to_current(virtual_acceleration);
    };
    // Conditional anti-windup in actuator coordinates. Inhibits integration
    // when either motor would saturate (conservative for a coupled controller).
    const Vec2 candidate = calculate(integral_error);
    if (enable_integral &&
        std::abs(candidate[0]) < current_limit &&
        std::abs(candidate[1]) < current_limit)
    {
      for (std::size_t i = 0; i < 2; ++i) {
        integral_error[i] += error[i] * dt;
      }
    }
    return calculate(integral_error);
  }
};

}  // namespace mimo
}  // namespace zlac8015d_hardware

#endif  // ZLAC8015D_MIMO_CONTROL_HPP_

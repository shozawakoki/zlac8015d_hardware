#ifndef ZLAC8015D_MIMO_CONTROL_HPP_
#define ZLAC8015D_MIMO_CONTROL_HPP_

// MIMO current-command controllers for a differential-drive robot.
// Physical wheel order is [left, right]; body order is [v, yaw_rate].
// Positive yaw is left/CCW: yaw_rate = r / track * (right - left).
// No ROS dependency: can be unit tested using a standard C++17 compiler.

#include <array>
#include <algorithm>
#include <cstddef>
#include <cmath>
#include <stdexcept>

namespace zlac8015d_hardware
{
namespace mimo
{

using Vec2 = std::array<double, 2>;
using Mat2 = std::array<Vec2, 2>;

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
  bool initialized{false};

  void reset()
  {
    integral_error = {{0.0, 0.0}};
    filtered_derivative = {{0.0, 0.0}};
    previous_measured = {{0.0, 0.0}};
    initialized = false;
  }

  Vec2 step(
    const DifferentialDriveModel & model, const Vec2 & reference,
    const Vec2 & measured, const double dt, const Vec2 & kp,
    const Vec2 & ki, const Vec2 & kd, const double derivative_tau,
    const double current_limit, const Vec2 & acceleration_feedforward = {{0.0, 0.0}},
    const bool enable_integral = true)
  {
    if (!initialized) {
      previous_measured = measured;
      initialized = true;
    }
    const double alpha = dt / (derivative_tau + dt);
    Vec2 error{};
    for (std::size_t i = 0; i < 2; ++i) {
      error[i] = reference[i] - measured[i];
      const double derivative = (measured[i] - previous_measured[i]) / dt;
      filtered_derivative[i] += alpha * (derivative - filtered_derivative[i]);
    }
    previous_measured = measured;
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

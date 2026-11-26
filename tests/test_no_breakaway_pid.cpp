// Offline C++17 test: no CAN, ROS, or motor commands.
// Tests the existing body-frame MIMO-PID law used by the continuous mode.
// The hardware-interface mode dispatch and stop-bypass must additionally
// be checked by code review and (after safety checks) in robot integration.
#include "zlac8015d_hardware/mimo_control.hpp"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>

using zlac8015d_hardware::mimo::DifferentialDriveModel;
using zlac8015d_hardware::mimo::Pid;
using zlac8015d_hardware::mimo::Vec2;

static Vec2 next_pid(
  Pid & pid, const DifferentialDriveModel & model,
  const Vec2 & ref, const Vec2 & measured,
  std::chrono::steady_clock::time_point sample_time)
{
  constexpr double dt = 0.005; // 200 Hz
  const Vec2 kp{{0.7, 0.7}};
  const Vec2 ki{{0.5, 0.5}};
  const Vec2 kd{{0.03, 0.03}};
  const Vec2 zero_ff{{0.0, 0.0}};
  return pid.step(model, ref, measured, dt, kp, ki, kd,
    0.05, 8.0, zero_ff, true, sample_time);
}

static double limited_current(
  double raw, double previous, double max_a, double rate_a_s, double dt)
{
  raw = std::max(-max_a, std::min(max_a, raw));
  const double delta = rate_a_s * dt;
  return std::max(-max_a, std::min(max_a,
    std::max(previous - delta, std::min(previous + delta, raw))));
}

int main()
{
  DifferentialDriveModel model(0.064, 0.354, 18.0, 18.0);
  const Vec2 zero_body{{0.0, 0.0}};
  const auto t0 = std::chrono::steady_clock::time_point{} +
    std::chrono::milliseconds(100);
  const auto t1 = t0 + std::chrono::milliseconds(10);

  // At standstill, MIMO-PID directly commands forward drive (no fixed
  // BREAKAWAY current, wait-to-track state, or startup feedforward).
  Pid straight;
  const auto forward_ref = model.wheel_to_body({{1.0, 1.0}});
  const auto initial = next_pid(straight, model, forward_ref, zero_body, t0);
  assert(initial[0] > 0.0 && initial[1] > 0.0);
  assert(straight.initialized);
  assert(!straight.derivative_updated); // initial observation is baseline
  // Same output protection as continuous MIMO-LADRC.
  assert(std::abs(limited_current(initial[0], 0.0, 8.0, 20.0, 0.005)) <= 0.1 + 1e-12);
  assert(std::abs(limited_current(initial[1], 0.0, 8.0, 20.0, 0.005)) <= 0.1 + 1e-12);

  // Straight-line slowdown is computed by PID with zero speed reference.
  // Preserve the controller state (do not call reset at transition).
  const auto positive_motion = model.wheel_to_body({{5.0, 5.0}});
  const auto stop_current = next_pid(straight, model, zero_body, positive_motion, t1);
  assert(stop_current[0] < 0.0 && stop_current[1] < 0.0);
  assert(straight.initialized);
  assert(straight.derivative_updated);
  assert(std::abs(straight.derivative_sample_dt_s - 0.010) < 1e-10);
  const double integral_after_stop = straight.integral_error[0];
  assert(integral_after_stop < straight.integral_error[1] + 1e-12);
  // Another step at zero reference uses the same integrator; no reset.
  next_pid(straight, model, zero_body, positive_motion, t1);
  assert(straight.initialized);
  assert(!straight.derivative_updated);
  assert(straight.integral_error[0] < integral_after_stop);

  // In-place turn at startup: the wheel current directions are opposed.
  Pid turn;
  const auto turn_ref = model.wheel_to_body({{-2.0, 2.0}});
  const auto turn_start = next_pid(turn, model, turn_ref, zero_body, t0);
  assert(turn_start[0] < 0.0 && turn_start[1] > 0.0);

  // In-place stop: PID (not an open-loop stop ramp) provides braking.
  Pid turn_stop_pid;
  const auto turn_motion = model.wheel_to_body({{-3.0, 3.0}});
  const auto turn_stop = next_pid(turn_stop_pid, model, zero_body, turn_motion, t0);
  assert(turn_stop[0] > 0.0 && turn_stop[1] < 0.0);

  // Conditional anti-windup when the 8 A requested-current cap would
  // be exceeded. This tests the existing PID algorithm, not a new gain.
  Pid saturated;
  const auto excessive_ref = model.wheel_to_body({{100.0, 100.0}});
  const auto big_current = next_pid(saturated, model, excessive_ref, zero_body, t0);
  assert(big_current[0] > 8.0 && big_current[1] > 8.0);
  assert(std::abs(saturated.integral_error[0]) < 1e-12);
  assert(std::abs(saturated.integral_error[1]) < 1e-12);

  std::cout << "Continuous no-BREAKAWAY MIMO-PID pure-model tests passed\n";
}

// Pure C++17 simulation test; no ROS, CAN interface or motor commands.
#include "zlac8015d_hardware/mimo_control.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using zlac8015d_hardware::mimo::DifferentialDriveModel;
using zlac8015d_hardware::mimo::Ladrc;
using zlac8015d_hardware::mimo::Vec2;

static double limit_current(
  double raw, double previous, double imax, double rate, double dt)
{
  raw = std::max(-imax, std::min(imax, raw));
  return std::max(
    -imax, std::min(imax,
      std::max(previous - rate * dt, std::min(previous + rate * dt, raw))));
}

int main()
{
  DifferentialDriveModel model(0.064, 0.354, 18.0, 18.0);
  const Vec2 stationary{{0.0, 0.0}};
  const double dt = 0.005;
  const double wc = 5.0;
  const double wo = 20.0;

  // Low-speed forward start (0.05 m/s) must use the ordinary LADRC law
  // without a forced 0.65 A BREAKAWAY current.
  Ladrc from_rest;
  from_rest.reset_to_measurement(model.wheel_to_body(stationary));
  const auto reference = model.wheel_to_body({{0.05 / 0.064, 0.05 / 0.064}});
  const auto initial = from_rest.step(
    model, reference, model.wheel_to_body(stationary), stationary, dt,
    {{wc, wc}}, {{wo, wo}});
  const double expected = wc * (0.05 / 0.064) / 18.0;
  assert(std::abs(initial[0] - expected) < 1.0e-10);
  assert(std::abs(initial[1] - expected) < 1.0e-10);
  assert(initial[0] < 0.65);
  // 20 A/s current slope -> first command max 0.10 A.
  assert(std::abs(limit_current(initial[0], 0.0, 1.0, 20.0, dt) - 0.1) < 1.0e-9);

  // A completely stationary "stiction" simulation: if measured speed is
  // always zero while current commands are applied, the LESO learns a
  // counteracting disturbance and increases effort within the clamp.
  Ladrc stuck;
  stuck.reset_to_measurement(model.wheel_to_body(stationary));
  Vec2 command{{0.0, 0.0}};
  bool exceeded_static_threshold = false;
  for (int i = 0; i < 400; ++i) {
    const Vec2 raw = stuck.step(
      model, reference, model.wheel_to_body(stationary), command, dt,
      {{wc, wc}}, {{wo, wo}});
    for (int wheel = 0; wheel < 2; ++wheel) {
      assert(std::isfinite(raw[wheel]));
      command[wheel] = limit_current(
        raw[wheel], command[wheel], 1.0, 20.0, dt);
      assert(std::abs(command[wheel]) <= 1.0 + 1.0e-12);
    }
    if (command[0] > 0.65 && command[1] > 0.65) {
      exceeded_static_threshold = true;
    }
  }
  assert(exceeded_static_threshold);

  // From rest, request a pure turn: both current signs must be correct.
  Ladrc turn;
  turn.reset_to_measurement(model.wheel_to_body(stationary));
  const Vec2 turn_ref = model.wheel_to_body({{+1.0, -1.0}});
  const Vec2 turn_current = turn.step(
    model, turn_ref, model.wheel_to_body(stationary), stationary, dt,
    {{wc, wc}}, {{wo, wo}});
  assert(turn_current[0] > 0.0);
  assert(turn_current[1] < 0.0);

  // Guard uses both body-coordinate errors and wheel-specific velocity.
  // Reproduce the measured left-wheel lock at ~8.5s in the supplied CSV:
  // desired body [v,yaw]=[0.60,0.15], measured=[0.31,1.77].
  zlac8015d_hardware::mimo::CoordinatedStallGuard guard;
  const Vec2 blocked_wheel_reference{{8.92, 9.77}};
  const Vec2 blocked_wheel_measured{{0.0, 9.77}};
  for (int i = 0; i < 29; ++i) {
    assert(!guard.update(
      blocked_wheel_reference, blocked_wheel_measured,
      0.60, 0.31, 0.15, 1.77, dt));
  }
  assert(guard.update(
    blocked_wheel_reference, blocked_wheel_measured,
    0.60, 0.31, 0.15, 1.77, dt));
  assert(guard.latched);
  assert(guard.stalled_wheel == 0);
  assert(guard.update(
    blocked_wheel_reference, {{4.0, 9.77}},
    0.60, 0.31, 0.15, 1.77, dt)); // remains latched
  guard.reset();
  assert(!guard.latched);
  // No false trip during normal straight/turn speeds with no large errors.
  for (int i = 0; i < 500; ++i) {
    assert(!guard.update(
      {{8.92, 9.77}}, {{8.90, 9.75}},
      0.60, 0.598, 0.15, 0.154, dt));
  }
  // No trip if only one wheel was commanded to move.
  for (int i = 0; i < 100; ++i) {
    assert(!guard.update(
      {{0.0, 9.77}}, {{0.0, 9.77}},
      0.30, 0.31, 1.77, 1.77, dt));
  }
  // No trip on a normal synchronized start where both wheels are still.
  for (int i = 0; i < 100; ++i) {
    assert(!guard.update(
      {{8.92, 9.77}}, {{0.0, 0.0}},
      0.60, 0.0, 0.15, 0.0, dt));
  }
  // No requirement for actual wheel movement before the LADRC step.
  // Saturation and slew-rate protection still apply.
  std::cout << "No-BREAKAWAY LADRC pure-model tests passed\n";
}

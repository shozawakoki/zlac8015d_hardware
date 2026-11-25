// Pure C++17 simulation test; no ROS, CAN interface or motor commands.
#include "zlac8015d_hardware/mimo_control.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

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

  // Wheel peripheral overspeed at 2.0 m/s:
  // omega_limit = 2.0/0.064 = 31.25 rad/s.
  // Trigger at the threshold on EITHER wheel, in either direction.
  using zlac8015d_hardware::mimo::wheel_overspeed;
  const double omega_limit = 2.0 / 0.064;
  assert(!wheel_overspeed({{omega_limit - 0.01, 0.0}}, 0.064));
  assert(!wheel_overspeed({{0.0, -omega_limit + 0.01}}, 0.064));
  assert(wheel_overspeed({{omega_limit, 0.0}}, 0.064));
  assert(wheel_overspeed({{0.0, -omega_limit}}, 0.064));
  assert(wheel_overspeed({{-35.0, 1.0}}, 0.064));
  assert(!wheel_overspeed({{+20.0, -20.0}}, 0.064));
  assert(wheel_overspeed({{0.0, 1.0}}, -0.064));
  // Invalid encoder feedback must fail closed.
  assert(wheel_overspeed(
    {{std::numeric_limits<double>::quiet_NaN(), 0.0}}, 0.064));

  // Normal LADRC still has the 8 A command ceiling with 20 A/s slew.
  assert(std::abs(limit_current(10.0, 7.9, 8.0, 20.0, dt) - 8.0) < 1.0e-12);
  assert(std::abs(limit_current(-10.0, -7.9, 8.0, 20.0, dt) + 8.0) < 1.0e-12);

  // The observer's model input is exactly the supplied current.
  // A smaller *measured* current must not be silently replaced with the
  // larger commanded current when the new mode selects TPDO1 feedback.
  Ladrc actual_input_eso;
  Ladrc command_input_eso;
  actual_input_eso.reset_to_measurement(model.wheel_to_body(stationary));
  command_input_eso.reset_to_measurement(model.wheel_to_body(stationary));
  const Vec2 actual_current{{0.2, -0.1}};
  const Vec2 commanded_current{{0.8, -0.5}};
  const auto actual_u = actual_input_eso.step(
    model, reference, model.wheel_to_body(stationary),
    actual_current, dt, {{wc, wc}}, {{wo, wo}});
  const auto commanded_u = command_input_eso.step(
    model, reference, model.wheel_to_body(stationary),
    commanded_current, dt, {{wc, wc}}, {{wo, wo}});
  const auto expected_acceleration_actual =
    zlac8015d_hardware::mimo::multiply(model.B, actual_current);
  const auto expected_acceleration_commanded =
    zlac8015d_hardware::mimo::multiply(model.B, commanded_current);
  for (int axis = 0; axis < 2; ++axis) {
    assert(std::abs(actual_input_eso.z1[axis] -
      dt * expected_acceleration_actual[axis]) < 1.0e-12);
    assert(std::abs(command_input_eso.z1[axis] -
      dt * expected_acceleration_commanded[axis]) < 1.0e-12);
  }
  assert(std::abs(actual_u[0] - commanded_u[0]) > 1.0e-6);
  assert(std::abs(actual_u[1] - commanded_u[1]) > 1.0e-6);

  // A zero-velocity reference must still be processed by MIMO-LADRC.
  // No normal-stop feedforward or open-loop zero-current hold is used
  // in the measured-current experiment. From positive motion the command
  // must initially be a braking current, subject to the existing limits.
  const Vec2 zero_body_reference = model.wheel_to_body(stationary);
  const Vec2 rolling_body = model.wheel_to_body({{6.0, 6.0}});
  Ladrc closed_loop_stop;
  closed_loop_stop.reset_to_measurement(rolling_body);
  const auto braking = closed_loop_stop.step(
    model, zero_body_reference, rolling_body, stationary,
    dt, {{wc, wc}}, {{wo, wo}});
  assert(braking[0] < 0.0 && braking[1] < 0.0);
  assert(closed_loop_stop.initialized);
  assert(std::abs(limit_current(braking[0], 0.0, 8.0, 20.0, dt)) <= 0.1 + 1e-12);

  // Continue the same ESO instance across the zero-reference transition.
  const auto updated_measured_body = model.wheel_to_body({{3.0, 3.0}});
  const auto z1_before = closed_loop_stop.z1;
  const auto braking_next = closed_loop_stop.step(
    model, zero_body_reference, updated_measured_body, stationary,
    dt, {{wc, wc}}, {{wo, wo}});
  assert(std::isfinite(braking_next[0]) && std::isfinite(braking_next[1]));
  assert(closed_loop_stop.initialized);
  assert(std::abs(closed_loop_stop.z1[0] - z1_before[0]) > 1e-9);

  // For a pure turn, zero target should brake the two wheels in
  // opposite directions (not inject a preprogrammed stop ramp).
  const Vec2 turning_body = model.wheel_to_body({{-4.0, 4.0}});
  Ladrc closed_loop_turn_stop;
  closed_loop_turn_stop.reset_to_measurement(turning_body);
  const auto turn_braking = closed_loop_turn_stop.step(
    model, zero_body_reference, turning_body, stationary,
    dt, {{wc, wc}}, {{wo, wo}});
  assert(turn_braking[0] > 0.0 && turn_braking[1] < 0.0);

  // No requirement for actual wheel movement before the LADRC step.
  // Saturation and slew-rate protection still apply.
  std::cout << "No-BREAKAWAY LADRC pure-model tests passed\n";
}

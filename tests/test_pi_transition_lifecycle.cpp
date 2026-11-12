#include <cassert>
#include <cmath>
#include <iostream>

#include "zlac8015d_hardware/pi_transition_lifecycle.hpp"

using namespace zlac8015d_hardware;

struct WheelState
{
  bool pi_transitioning{false};
  double pi_transition_elapsed_s{0.0};
  double pi_transition_start_current_a{0.0};
  double pi_transition_start_speed_rad_s{0.0};
};

bool near(double x, double y)
{
  return std::abs(x - y) < 1.0e-9;
}

int main()
{
  WheelState wheel;
  const double reference = 0.78125;
  const double dt = 0.005;
  const double duration = 0.10;

  // BREAKAWAY -> TRANSITION.
  begin_pi_transition(wheel, 0.95, 0.40);
  assert(wheel.pi_transitioning);
  assert(near(wheel.pi_transition_elapsed_s, 0.0));
  assert(near(wheel.pi_transition_start_current_a, 0.95));
  const double first = advance_pi_transition(
    wheel, 0.12, 0.40, reference, dt, duration, +1, false);
  assert(wheel.pi_transitioning);
  assert(first > 0.12 && first < 0.95);

  // TRANSITION -> TRACK by speed threshold.
  const double at_speed = advance_pi_transition(
    wheel, 0.12, 0.8 * reference, reference, dt, duration, +1, false);
  assert(!wheel.pi_transitioning);
  assert(near(at_speed, 0.12));
  assert(near(wheel.pi_transition_elapsed_s, 0.0));
  // TRACK remains PI-only; transition cannot restart spontaneously.
  assert(near(advance_pi_transition(
    wheel, 0.12, 0.30, reference, dt, duration, +1, false), 0.12));
  assert(!wheel.pi_transitioning);

  // TRANSITION -> TRACK by time deadline.
  begin_pi_transition(wheel, 0.95, 0.40);
  for (int i = 0; i < 30 && wheel.pi_transitioning; ++i) {
    (void)advance_pi_transition(
      wheel, 0.12, 0.40, reference, dt, duration, +1, false);
  }
  assert(!wheel.pi_transitioning);

  // Stop request cancels the phase and does not keep assisted current.
  begin_pi_transition(wheel, 0.95, 0.40);
  const double stop = advance_pi_transition(
    wheel, 0.12, 0.40, reference, dt, duration, +1, true);
  assert(near(stop, 0.12));
  assert(!wheel.pi_transitioning);

  // Braking request cancels assist, preserving opposite-sign torque.
  begin_pi_transition(wheel, 0.95, 0.40);
  const double brake = advance_pi_transition(
    wheel, -0.15, 0.40, reference, dt, duration, +1, false);
  assert(near(brake, -0.15));
  assert(!wheel.pi_transitioning);

  // Direction reversal -> BREAKAWAY reset, then reverse transition.
  begin_pi_transition(wheel, 0.95, 0.40);
  cancel_pi_transition(wheel);
  assert(!wheel.pi_transitioning);
  begin_pi_transition(wheel, -0.95, 0.40);
  const double reverse = advance_pi_transition(
    wheel, -0.12, 0.40, reference, dt, duration, -1, false);
  assert(wheel.pi_transitioning && reverse < -0.12);

  // Coordinated two-wheel transition and synchronized cancellation.
  WheelState left, right;
  begin_pi_transition(left, 0.95, 0.40);
  begin_pi_transition(right, 0.85, 0.40);
  assert(left.pi_transitioning && right.pi_transitioning);
  cancel_pi_transition(left);
  cancel_pi_transition(right);
  assert(!left.pi_transitioning && !right.pi_transitioning);

  // Repeated-stall lockout and zero-ref shutdown call the same
  // production cancellation helper (the lockout detector is separate).
  begin_pi_transition(wheel, 0.95, 0.40);
  cancel_pi_transition(wheel);
  assert(!wheel.pi_transitioning);
  assert(near(wheel.pi_transition_start_current_a, 0.0));

  std::cout << "PASS: PI transition lifecycle tests\n";
}

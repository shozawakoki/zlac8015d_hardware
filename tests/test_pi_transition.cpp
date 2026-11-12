#include <cassert>
#include <cmath>
#include <iostream>

#include "zlac8015d_hardware/pi_transition.hpp"

using zlac8015d_hardware::compute_pi_transition;

void close_to(const double got, const double expected)
{
  assert(std::abs(got - expected) < 1.0e-9);
}

int main()
{
  // 0.78125 rad/s target, 0.95 A BREAKAWAY, 0.12 A PI request.
  const double ref = 0.78125;
  const auto early = compute_pi_transition(
    0.12, 0.95, 0.40, 0.40, ref, 0.005, 0.10, +1, false);
  assert(early.active);
  assert(early.requested_current_a > 0.12);
  assert(early.requested_current_a < 0.95);

  const auto later = compute_pi_transition(
    0.12, 0.95, 0.40, 0.50, ref, 0.05, 0.10, +1, false);
  assert(later.active);
  assert(later.requested_current_a < early.requested_current_a);

  // Never drive harder than ordinary PI if PI already exceeds BREAKAWAY.
  const auto high_pi = compute_pi_transition(
    1.20, 0.95, 0.40, 0.45, ref, 0.005, 0.10, +1, false);
  assert(high_pi.active);
  close_to(high_pi.requested_current_a, 1.20);

  // Exit at 80% reference, time limit, or on braking/normal stop.
  const auto cutoff = compute_pi_transition(
    0.12, 0.95, 0.40, 0.8 * ref, ref, 0.02, 0.10, +1, false);
  assert(!cutoff.active);
  close_to(cutoff.requested_current_a, 0.12);
  const auto timed = compute_pi_transition(
    0.12, 0.95, 0.40, 0.40, ref, 0.10, 0.10, +1, false);
  assert(!timed.active);
  const auto braking = compute_pi_transition(
    -0.20, 0.95, 0.40, 0.40, ref, 0.005, 0.10, +1, false);
  assert(!braking.active);
  close_to(braking.requested_current_a, -0.20);
  const auto stopping = compute_pi_transition(
    0.12, 0.95, 0.40, 0.40, ref, 0.005, 0.10, +1, true);
  assert(!stopping.active);
  close_to(stopping.requested_current_a, 0.12);

  // Reverse-direction request: sign and speed projection are consistent.
  const auto reverse = compute_pi_transition(
    -0.12, -0.95, 0.40, 0.45, ref, 0.005, 0.10, -1, false);
  assert(reverse.active);
  assert(reverse.requested_current_a < -0.12);
  assert(reverse.requested_current_a > -0.95);

  // If already near target at handoff, provide no assist.
  const auto at_cutoff = compute_pi_transition(
    0.12, 0.95, 0.70, 0.70, ref, 0.005, 0.10, +1, false);
  assert(!at_cutoff.active);

  std::cout << "PASS: PI transition pure-function regression tests\n";
}

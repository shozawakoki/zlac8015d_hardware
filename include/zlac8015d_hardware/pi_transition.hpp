#ifndef ZLAC8015D_PI_TRANSITION_HPP_
#define ZLAC8015D_PI_TRANSITION_HPP_

#include <algorithm>
#include <cmath>

namespace zlac8015d_hardware
{

struct PiTransitionResult
{
  double requested_current_a;
  bool active;
};

// Pure transition assist: no ROS, CAN, or state mutation.
// Caller retains the authoritative current-cap, slew-rate and stop logic.
inline PiTransitionResult compute_pi_transition(
  const double raw_current_a,
  const double start_current_a,
  const double start_speed_in_direction_rad_s,
  const double speed_in_direction_rad_s,
  const double absolute_reference_rad_s,
  const double elapsed_s,
  const double duration_s,
  const int direction,
  const bool normal_stop_active)
{
  if (normal_stop_active || direction == 0 ||
      direction * raw_current_a <= 0.0 || elapsed_s >= duration_s ||
      duration_s <= 0.0)
  {
    return {raw_current_a, false};
  }

  const double cutoff_speed = 0.8 * absolute_reference_rad_s;
  const double speed_headroom =
    cutoff_speed - start_speed_in_direction_rad_s;
  const double remaining_speed =
    cutoff_speed - speed_in_direction_rad_s;
  if (remaining_speed <= 0.0 || speed_headroom <= 1.0e-6) {
    return {raw_current_a, false};
  }

  const double time_weight = std::clamp(1.0 - elapsed_s / duration_s, 0.0, 1.0);
  const double speed_weight =
    std::clamp(remaining_speed / speed_headroom, 0.0, 1.0);
  const double pi_drive_a = direction * raw_current_a;
  const double breakaway_drive_a =
    std::max(0.0, direction * start_current_a);
  const double assist_a = time_weight * speed_weight *
    std::max(0.0, breakaway_drive_a - pi_drive_a);

  return {raw_current_a + direction * assist_a, true};
}

}  // namespace zlac8015d_hardware

#endif  // ZLAC8015D_PI_TRANSITION_HPP_

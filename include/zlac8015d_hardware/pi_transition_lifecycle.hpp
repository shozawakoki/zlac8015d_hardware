#ifndef ZLAC8015D_PI_TRANSITION_LIFECYCLE_HPP_
#define ZLAC8015D_PI_TRANSITION_LIFECYCLE_HPP_

#include <cmath>
#include "zlac8015d_hardware/pi_transition.hpp"

namespace zlac8015d_hardware
{

// Templated on the wheel state to avoid ROS dependencies in offline tests.
// These helpers are used by the production hardware plugin.
template<class State>
void cancel_pi_transition(State & state)
{
  state.pi_transitioning = false;
  state.pi_transition_elapsed_s = 0.0;
  state.pi_transition_start_current_a = 0.0;
  state.pi_transition_start_speed_rad_s = 0.0;
}

template<class State>
void begin_pi_transition(
  State & state, const double last_current_a,
  const double speed_in_command_direction_rad_s)
{
  state.pi_transitioning = true;
  state.pi_transition_elapsed_s = 0.0;
  state.pi_transition_start_current_a = last_current_a;
  state.pi_transition_start_speed_rad_s =
    speed_in_command_direction_rad_s;
}

template<class State>
double advance_pi_transition(
  State & state, const double raw_current_a,
  const double measured_speed_in_direction_rad_s,
  const double absolute_reference_rad_s, const double dt,
  const double duration_s, const int direction,
  const bool suppress_transition)
{
  if (!state.pi_transitioning) {
    return raw_current_a;
  }
  state.pi_transition_elapsed_s += dt;
  const auto result = compute_pi_transition(
    raw_current_a,
    state.pi_transition_start_current_a,
    state.pi_transition_start_speed_rad_s,
    measured_speed_in_direction_rad_s,
    absolute_reference_rad_s,
    state.pi_transition_elapsed_s,
    duration_s,
    direction,
    suppress_transition);
  if (!result.active) {
    cancel_pi_transition(state);
  }
  return result.requested_current_a;
}

}  // namespace zlac8015d_hardware

#endif  // ZLAC8015D_PI_TRANSITION_LIFECYCLE_HPP_

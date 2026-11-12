// Simplified offline closed-loop comparison: NOT the production state machine.
// Shares the *production* pi_transition_lifecycle.hpp helper.
// No ROS, CAN or motor connection. Outputs CSV for offline analysis.
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "zlac8015d_hardware/pi_transition_lifecycle.hpp"

namespace {
constexpr double dt = 0.005;
constexpr double wheel_ref = 0.05 / 0.064;  // 0.78125 rad/s
constexpr double kp = 0.4;
constexpr double ki = 0.2;
constexpr double b0 = 18.0;
constexpr double delay_s = 0.023;
constexpr double current_limit = 4.0;
constexpr double slew_a_per_s = 20.0;
constexpr double max_wheel_speed_abort = 1.2 * 1.2;  // mirrors 1.2 rad/s monitor +20%
constexpr double pi_floor = 0.20;
constexpr int start_samples = 6;
constexpr double start_speed = 0.30;
constexpr double startup_current = 0.65;
constexpr double startup_increment = 0.10;
constexpr double startup_max = 2.0;
constexpr int startup_increment_samples = 20;

double limit(double x, double lo, double hi)
{
  return std::clamp(x, lo, hi);
}

enum class Mode { legacy, bumpless, three_stage };

const char * mode_name(Mode mode)
{
  switch (mode) {
    case Mode::legacy: return "legacy";
    case Mode::bumpless: return "bumpless";
    case Mode::three_stage: return "three_stage";
  }
  return "unknown";
}

struct Wheel
{
  double speed{0.0};
  double last_current{0.0};
  double integral{0.0};
  double breakaway_current{startup_current};
  int start_count{0};
  int breakaway_samples{0};
  bool track{false};
  bool pi_initialized{false};
  bool pi_transitioning{false};
  double pi_transition_elapsed_s{0.0};
  double pi_transition_start_current_a{0.0};
  double pi_transition_start_speed_rad_s{0.0};
  std::deque<double> delayed_current;
};

struct Outcome
{
  std::string mode;
  double friction_current{0.0};
  double rmse_rad_s{0.0};
  double maximum_speed{0.0};
  double maximum_current{0.0};
  double speed_at_stop{0.0};
  double final_speed{0.0};
  double transition_end_time{-1.0};
  int active_samples{0};
  bool overspeed{false};
};

double desired_speed(double t)
{
  return t >= 1.0 && t < 3.0 ? wheel_ref : 0.0;
}

// Educational plant: G(s) = 18/s exp(-0.023s) when friction_current = 0.
// Coulomb/static friction is approximated as a threshold torque, not an
// identified physical model. Delay is rounded to sample-grid steps.
void plant_step(Wheel & wheel, double cmd_current, double friction)
{
  wheel.delayed_current.push_back(cmd_current);
  const double delayed = wheel.delayed_current.front();
  wheel.delayed_current.pop_front();
  double acceleration = 0.0;
  if (std::abs(wheel.speed) <= 1.0e-8) {
    if (std::abs(delayed) > friction) {
      acceleration = b0 * (delayed - std::copysign(friction, delayed));
    }
  } else {
    const double net = delayed - std::copysign(friction, wheel.speed);
    acceleration = b0 * net;
  }
  const double before = wheel.speed;
  wheel.speed += dt * acceleration;
  if (before * wheel.speed < 0.0 &&
      std::abs(delayed) <= friction) {
    wheel.speed = 0.0;
  }
}

double controller_step(
  Wheel & wheel, Mode mode, double target, double friction)
{
  (void)friction;
  if (target <= 1.0e-9) {
    zlac8015d_hardware::cancel_pi_transition(wheel);
    wheel.track = false;
    wheel.start_count = 0;
    wheel.breakaway_samples = 0;
    wheel.breakaway_current = startup_current;
    wheel.pi_initialized = false;
    wheel.integral = 0.0;
    // Simple stop surrogate, NOT actual controller normal-stop ramp.
    const double requested = limit(-0.9 * wheel.speed, -current_limit, current_limit);
    wheel.last_current = limit(
      requested, wheel.last_current - slew_a_per_s * dt,
      wheel.last_current + slew_a_per_s * dt);
    return wheel.last_current;
  }

  if (!wheel.track) {
    if (wheel.speed >= start_speed) {
      ++wheel.start_count;
    } else {
      wheel.start_count = 0;
    }
    if (wheel.speed < start_speed &&
        wheel.breakaway_samples > 0 &&
        wheel.breakaway_samples % startup_increment_samples == 0) {
      wheel.breakaway_current = std::min(
        startup_max, wheel.breakaway_current + startup_increment);
    }
    ++wheel.breakaway_samples;
    wheel.last_current = wheel.breakaway_current;
    if (wheel.start_count >= start_samples) {
      wheel.track = true;
      wheel.pi_initialized = false;
      if (mode == Mode::three_stage) {
        zlac8015d_hardware::begin_pi_transition(
          wheel, wheel.last_current, wheel.speed);
      }
    }
    return wheel.last_current;
  }

  const double error = target - wheel.speed;
  const double p = kp * error;
  if (!wheel.pi_initialized) {
    wheel.integral = mode == Mode::bumpless ?
      limit(wheel.last_current - p, -current_limit, current_limit) : 0.0;
    wheel.pi_initialized = true;
  }
  const double tentative = p + wheel.integral;
  if (!((tentative >= current_limit && error > 0.0) ||
        (tentative <= -current_limit && error < 0.0))) {
    wheel.integral = limit(wheel.integral + ki * error * dt,
                           -current_limit, current_limit);
  }
  const double raw_pi = p + wheel.integral;
  const double requested = zlac8015d_hardware::advance_pi_transition(
    wheel, raw_pi, wheel.speed, target, dt, 0.10, +1, false);
  double current = limit(requested, -current_limit, current_limit);
  current = limit(current,
    wheel.last_current - slew_a_per_s * dt,
    wheel.last_current + slew_a_per_s * dt);
  // Track current floor as in the hardware (applied only for positive raw PI).
  if (raw_pi >= 0.0 && current >= 0.0 && current < pi_floor) {
    current = pi_floor;
  }
  current = limit(current, -current_limit, current_limit);
  wheel.last_current = current;
  return current;
}

Outcome simulate(Mode mode, double friction, const std::string & csv_path)
{
  std::ofstream csv(csv_path);
  if (!csv) {
    throw std::runtime_error("Cannot open output CSV: " + csv_path);
  }
  csv << std::setprecision(10);
  csv << "time_s,reference_rad_s,speed_rad_s,command_current_a,"
         "mode,track,transition,integral_a,friction_a\n";
  Wheel wheel;
  // 23 ms rounded to 5 samples of 5 ms = 25 ms.
  const std::size_t delay_steps =
    static_cast<std::size_t>(std::llround(delay_s / dt));
  wheel.delayed_current.assign(delay_steps, 0.0);
  Outcome out;
  out.mode = mode_name(mode);
  out.friction_current = friction;
  double square_error = 0.0;
  bool previously_transitioning = false;

  for (int tick = 0; tick < 1000; ++tick) {
    const double t = tick * dt;
    const double target = desired_speed(t);
    const double current = controller_step(wheel, mode, target, friction);
    csv << t << ',' << target << ',' << wheel.speed << ','
        << current << ',' << mode_name(mode) << ','
        << int(wheel.track) << ',' << int(wheel.pi_transitioning) << ','
        << wheel.integral << ',' << friction << '\n';

    out.maximum_speed = std::max(out.maximum_speed, std::abs(wheel.speed));
    out.maximum_current = std::max(out.maximum_current, std::abs(current));
    if (previously_transitioning && !wheel.pi_transitioning &&
        out.transition_end_time < 0.0) {
      out.transition_end_time = t;
    }
    previously_transitioning = wheel.pi_transitioning;

    if (t >= 1.0 && t < 3.0) {
      square_error += (target - wheel.speed) * (target - wheel.speed);
      ++out.active_samples;
    }
    if (t >= 3.0 && t < 3.0 + dt) {
      out.speed_at_stop = wheel.speed;
    }
    if (std::abs(wheel.speed) > max_wheel_speed_abort) {
      out.overspeed = true;
      // Do not stop the *offline* simulation; retain the full response
      // for diagnosis. This is NOT the physical abort path.
    }
    plant_step(wheel, current, friction);
  }
  out.final_speed = wheel.speed;
  out.rmse_rad_s = std::sqrt(square_error / out.active_samples);
  return out;
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string dir = argc > 1 ? argv[1] : "/tmp";
  std::cout << "mode,friction_A,RMSE_rad_s,peak_speed_rad_s,"
               "peak_current_A,speed_at_stop_rad_s,final_speed_rad_s,"
               "transition_end_s,overspeed_threshold_crossed\n";
  for (const double friction : {0.0, 0.2, 0.4}) {
    for (const auto mode : {Mode::legacy, Mode::bumpless, Mode::three_stage}) {
      const std::string file = dir + "/pi_" + mode_name(mode) +
        "_friction_" + std::to_string(int(friction * 100)) + ".csv";
      const auto result = simulate(mode, friction, file);
      std::cout << result.mode << ',' << friction << ','
                << result.rmse_rad_s << ',' << result.maximum_speed << ','
                << result.maximum_current << ',' << result.speed_at_stop << ','
                << result.final_speed << ',' << result.transition_end_time << ','
                << (result.overspeed ? "YES" : "NO") << '\n';
    }
  }
}

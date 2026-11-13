// Standalone MIMO tests (from the package root).
// Build: g++ -std=c++17 -Wall -Wextra -pedantic -Iinclude tests/test_mimo_control.cpp -o /tmp/test_mimo_control
// Run:   /tmp/test_mimo_control
#include "zlac8015d_hardware/mimo_control.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using zlac8015d_hardware::mimo::DifferentialDriveModel;
using zlac8015d_hardware::mimo::Ladrc;
using zlac8015d_hardware::mimo::Pid;
using zlac8015d_hardware::mimo::Vec2;
static bool close(double a, double b, double eps=1.0e-10) {
  return std::abs(a-b) < eps;
}
int main()
{
  // Actual wheel convention: left positive and right positive means forward.
  DifferentialDriveModel model(0.064, 0.354, 18.0, 18.0);
  const Vec2 wheels{{4.0, 6.0}};
  const auto body=model.wheel_to_body(wheels);
  assert(close(body[0], 0.32));
  assert(close(body[1], 0.064/0.354*2.0)); // right faster -> CCW
  const auto recovered=model.body_to_wheel(body);
  assert(close(recovered[0], wheels[0]));
  assert(close(recovered[1], wheels[1]));
  const Vec2 currents{{0.5, 0.8}};
  const auto body_acc=zlac8015d_hardware::mimo::multiply(model.B,currents);
  const auto again=model.acceleration_to_current(body_acc);
  assert(close(again[0], currents[0]));
  assert(close(again[1], currents[1]));
  
  // Standard LADRC: straight reference, right wheel overspeeding.
  const auto straight=model.wheel_to_body({{5.0,5.0}});
  Ladrc ladrc;
  ladrc.reset_to_measurement(body);
  const auto command=ladrc.step(
    model,straight,body,{{0.0,0.0}},0.005,
    {{5.0,5.0}},{{20.0,20.0}});
  assert(command[0] > 0.0); // accelerate slow left
  assert(command[1] < 0.0); // brake fast right
  // Identical v/yaw bandwidth is equivalent to independent wheel P feedback
  // in this no-disturbance, correctly-initialized single-step case.
  assert(close(command[0], 5.0*(5.0-4.0)/18.0));
  assert(close(command[1], 5.0*(5.0-6.0)/18.0));
  Ladrc high_yaw;
  high_yaw.reset_to_measurement(body);
  const auto stronger=high_yaw.step(
    model,straight,body,{{0.0,0.0}},0.005,
    {{5.0,10.0}},{{20.0,20.0}});
  assert(stronger[0]>command[0]);
  assert(stronger[1]<command[1]);

  // MIMO PID equivalent to wheel PID under equal gains, zero I/D action.
  Pid pid;
  const auto pid_current=pid.step(
    model,straight,body,0.005,
    {{0.7,0.7}},{{0.0,0.0}},{{0.0,0.0}},0.05,8.0);
  assert(close(pid_current[0],0.7));
  assert(close(pid_current[1],-0.7));
  Pid yaw_pid;
  const auto pid_stronger=yaw_pid.step(
    model,straight,body,0.005,
    {{0.7,1.4}},{{0.0,0.0}},{{0.0,0.0}},0.05,8.0);
  assert(pid_stronger[0]>pid_current[0]);
  assert(pid_stronger[1]<pid_current[1]);
  // The D term must use only NEW TPDO0 samples and their actual receive
  // time difference, not the 200 Hz controller dt. P/I remain on the 200 Hz loop.
  using Clock = std::chrono::steady_clock;
  using Ms = std::chrono::milliseconds;
  const Clock::time_point t0 = Clock::time_point{} + Ms(1000);
  const Vec2 zero{{0.0, 0.0}};
  const Vec2 d_gain{{1.0, 1.0}};
  Pid sampled_pid;
  (void)sampled_pid.step(
    model, zero, zero, 0.005, zero, zero, d_gain, 0.05, 8.0,
    zero, true, t0);
  assert(close(sampled_pid.filtered_derivative[0], 0.0));
  assert(!sampled_pid.derivative_updated);
  assert(close(sampled_pid.derivative_sample_dt_s, 0.0));
  const Vec2 first_sample{{0.010, 0.020}};
  // A 200 Hz controller tick without a CAN sample must not update D.
  (void)sampled_pid.step(
    model, zero, zero, 0.005, zero, zero, d_gain, 0.05, 8.0,
    zero, true, t0);
  assert(close(sampled_pid.filtered_derivative[0], 0.0));
  // Fresh CAN measurement arrives 10 ms after the first measurement:
  // derivative=[1,2], filtered with alpha=10/(50+10)=1/6.
  (void)sampled_pid.step(
    model, zero, first_sample, 0.005, zero, zero, d_gain, 0.05, 8.0,
    zero, true, t0 + Ms(10));
  assert(close(sampled_pid.filtered_derivative[0], 1.0 / 6.0));
  assert(close(sampled_pid.filtered_derivative[1], 2.0 / 6.0));
  assert(sampled_pid.derivative_updated);
  assert(close(sampled_pid.derivative_sample_dt_s, 0.010));
  // A repeated controller step with the same TPDO0 timestamp holds D.
  (void)sampled_pid.step(
    model, zero, first_sample, 0.005, zero, zero, d_gain, 0.05, 8.0,
    zero, true, t0 + Ms(10));
  assert(close(sampled_pid.filtered_derivative[0], 1.0 / 6.0));
  assert(!sampled_pid.derivative_updated);
  assert(close(sampled_pid.derivative_sample_dt_s, 0.0));
  // A new TPDO0 frame with identical speed is still a new sample:
  // D input is zero, the low-pass derivative decays.
  (void)sampled_pid.step(
    model, zero, first_sample, 0.005, zero, zero, d_gain, 0.05, 8.0,
    zero, true, t0 + Ms(20));
  assert(close(sampled_pid.filtered_derivative[0], 5.0 / 36.0));
  assert(close(sampled_pid.filtered_derivative[1], 10.0 / 36.0));
  assert(sampled_pid.derivative_updated);
  assert(close(sampled_pid.derivative_sample_dt_s, 0.010));
  // Bursts less than 1 ms apart are rejected, preserving the baseline.
  const Vec2 burst_sample{{0.050, 0.050}};
  (void)sampled_pid.step(
    model, zero, burst_sample, 0.005, zero, zero, d_gain, 0.05, 8.0,
    zero, true, t0 + Ms(20) + std::chrono::microseconds(100));
  assert(close(sampled_pid.filtered_derivative[0], 5.0 / 36.0));
  assert(!sampled_pid.derivative_updated);
  // After a long gap, the derivative resets instead of a large D kick.
  (void)sampled_pid.step(
    model, zero, burst_sample, 0.005, zero, zero, d_gain, 0.05, 8.0,
    zero, true, t0 + Ms(300));
  assert(close(sampled_pid.filtered_derivative[0], 0.0));
  assert(close(sampled_pid.filtered_derivative[1], 0.0));
  assert(!sampled_pid.derivative_updated);
  sampled_pid.reset();
  assert(!sampled_pid.initialized);
  assert(close(sampled_pid.filtered_derivative[0], 0.0));

  // Regression: the recorded failed rotation had target L=+2.766,
  // R=-2.766 rad/s, but actual L=-5.582, R=-24.588 rad/s.
  // Both wheels are moving; requiring target-direction motion left the
  // controller in BREAKAWAY with persistent [+2,-2] A.
  using zlac8015d_hardware::mimo::moving_for_track;
  using zlac8015d_hardware::mimo::coordinated_ready_target;
  assert(moving_for_track(+2.766, -5.582, 0.30, true));
  assert(moving_for_track(-2.766, -24.588, 0.30, true));
  assert(!moving_for_track(+2.766, -5.582, 0.30, false));
  assert(moving_for_track(-2.766, -24.588, 0.30, false));
  assert(!moving_for_track(+2.766, -0.15, 0.30, true));
  assert(!moving_for_track(0.0, -24.588, 0.30, true));
  assert(!moving_for_track(
    +2.766, std::numeric_limits<double>::quiet_NaN(), 0.30, true));
  assert(!moving_for_track(+2.766, -5.582, 0.0, true));

  // Both wheels become eligible for the coordinated TRACK transition
  // after sustained samples rather than staying in BREAKAWAY forever.
  int samples_left = 0;
  int samples_right = 0;
  for (int tick=0; tick<5; ++tick) {
    samples_left = moving_for_track(+2.766, -5.582, 0.30, true) ?
      samples_left + 1 : 0;
    samples_right = moving_for_track(-2.766, -24.588, 0.30, true) ?
      samples_right + 1 : 0;
  }
  assert(samples_left >= 5 && samples_right >= 5);
  assert(close(coordinated_ready_target(0.75, 2.0, true), 0.75));
  assert(close(coordinated_ready_target(0.75, 2.0, false), 2.0));

  // On entering MIMO TRACK, a measured right-wheel overspeed must produce
  // a BRAKING (positive) right-wheel current instead of the old -2 A.
  const Vec2 recorded_ref_wheels{{2.7656, -2.7656}};
  const Vec2 recorded_meas_wheels{{-5.5816, -24.5882}};
  const auto recorded_ref = model.wheel_to_body(recorded_ref_wheels);
  const auto recorded_meas = model.wheel_to_body(recorded_meas_wheels);
  Ladrc recovered_track;
  recovered_track.reset_to_measurement(recorded_meas);
  const auto brake = recovered_track.step(
    model, recorded_ref, recorded_meas, {{0.0, 0.0}}, 0.005,
    {{5.0, 5.0}}, {{20.0, 20.0}});
  assert(brake[0] > 0.0);
  assert(brake[1] > 0.0); // right wheel must decelerate its negative rotation

  std::cout << "MIMO model/LADRC/PID checks passed\n";
}

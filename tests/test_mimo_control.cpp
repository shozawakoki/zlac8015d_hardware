// Standalone tests: g++ -std=c++17 -Wall -Wextra -pedantic -Iinclude \
//   tests/test_mimo_control.cpp -o /tmp/test_mimo_control && /tmp/test_mimo_control
#include "zlac8015d_hardware/mimo_control.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
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
  std::cout << "MIMO model/LADRC/PID checks passed\n";
}

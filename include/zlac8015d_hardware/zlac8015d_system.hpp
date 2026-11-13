#ifndef ZLAC8015D_SYSTEM_HPP_
#define ZLAC8015D_SYSTEM_HPP_

#include <chrono>
#include "zlac8015d_hardware/mimo_control.hpp"
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace zlac8015d_hardware
{

class Zlac8015dSystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct LadrcState
  {
    double z1{0.0};          // estimated wheel speed [rad/s]
    double z2{0.0};          // estimated total disturbance [rad/s^2]
    double b0{1.0};          // nominal input gain [(rad/s^2)/A]
    double last_current_cmd{0.0};
    bool initialized{false};

    // --------------------------------------------------------
    // BREAKAWAY / TRACK state
    // --------------------------------------------------------
    bool tracking{false};

    int start_count{0};
    int stall_count{0};
    // Limit repeated automatic starts within one nonzero-speed command.
    int stall_recovery_attempts{0};
    bool stall_lockout{false};

    int breakaway_count{0};
    double breakaway_target{0.65};

    int track_hold_count{0};
    double track_hold_start{0.20};

    // +1 forward, -1 reverse, 0 stopped/uninitialized
    int direction_sign{0};
  };

  // Integral and derivative states used only in PID current mode.
  struct PidState
  {
    double integral_current{0.0}; // integral output [A]
    double filtered_velocity_derivative{0.0}; // [rad/s^2]
    double previous_velocity{0.0}; // [rad/s]
    bool initialized{false};
  };

  enum class ControlMode
  {
    VELOCITY,
    LADRC_TORQUE,
    PID_TORQUE,
    MIMO_LADRC_TORQUE,
    MIMO_PID_TORQUE
  };

  // CAN socket / CANopen configuration
  int can_socket_{-1};
  std::string can_interface_{"can0"};
  uint8_t node_id_{1};

  bool send_can_frame(uint32_t can_id, const std::vector<uint8_t> & data);
  bool configure_feedback_pdos();
  bool configure_velocity_rpdo();
  bool configure_torque_rpdo();
  void send_zero_command();

  // LADRC helper
  double compute_current_command(
    LadrcState & state,
    PidState & pid_state,
    double velocity_reference,
    double velocity_measured,
    double current_measured,
    double dt,
    bool allow_track_transition,
    bool mimo_torque_mode,
    bool use_mimo_raw,
    double mimo_raw_current);
  static double clamp(double value, double lower, double upper);

  // ros2_control command / state
  std::vector<double> hw_commands_;    // target wheel velocity [rad/s]
  std::vector<double> hw_positions_;   // encoder-based wheel position [rad]
  std::vector<double> hw_velocities_;  // actual wheel velocity [rad/s]
  std::vector<double> hw_efforts_;     // actual motor current [A], ROS-positive forward

  // Controller configuration
  ControlMode control_mode_{ControlMode::VELOCITY};
  LadrcState left_ladrc_;
  LadrcState right_ladrc_;
  PidState left_pid_;
  PidState right_pid_;
  // Two-input/two-output controllers operating in body coordinates [v, yaw_rate].
  mimo::DifferentialDriveModel mimo_model_{0.064, 0.354, 18.0, 18.0};
  mimo::Ladrc mimo_ladrc_;
  mimo::Pid mimo_pid_;
  bool mimo_was_ready_{false};
  double mimo_wheel_radius_m_{0.064};
  double mimo_wheel_separation_m_{0.354};
  mimo::Vec2 mimo_wc_{{5.0, 5.0}};
  mimo::Vec2 mimo_wo_{{20.0, 20.0}};
  mimo::Vec2 mimo_pid_kp_{{0.7, 0.7}};
  mimo::Vec2 mimo_pid_ki_{{0.5, 0.5}};
  mimo::Vec2 mimo_pid_kd_{{0.03, 0.03}};
  double pid_kp_{0.7};  // [A/(rad/s)]
  double pid_ki_{0.5};  // [A/rad]
  double pid_kd_{0.03}; // [A*s/rad]
  double pid_derivative_filter_tau_s_{0.05}; // [s]
  // Tunable start/recovery thresholds. Interpret step_samples against the
  // ros2_control update rate (nominally 200 Hz); no changes while running.
  double breakaway_start_a_{0.65};
  double breakaway_step_a_{0.10};
  double breakaway_max_a_{2.00};
  int breakaway_step_samples_{20};
  double breakaway_start_speed_rad_s_{0.30};
  int breakaway_start_samples_{6};
  double stall_speed_rad_s_{0.10};
  int stall_samples_{80};
  double stall_min_drive_current_a_{0.20};
  int sync_track_count_{0};
  double controller_bandwidth_{5.0};       // wc [rad/s]
  double observer_bandwidth_{20.0};        // wo [rad/s]
  double current_limit_a_{2.0};            // absolute LADRC output limit [A]
  double current_rate_limit_a_per_s_{20.0};
  double feedback_timeout_s_{0.030};

  // Feedback watchdog
  bool velocity_feedback_received_{false};
  bool current_feedback_received_{false};
  std::chrono::steady_clock::time_point last_velocity_feedback_time_{};
  bool feedback_timeout_reported_{false};

  // True while the ZLAC8015D is held in CiA402 Quick Stop state.
  bool quick_stop_active_{false};

  // True after Quick Stop has reached the first-stop region.
  // While the wheel references remain zero, keep zero current
  // without re-entering Quick Stop.
  bool zero_stop_hold_{false};

  // Normal-stop velocity ramp state.
  // Normal cmd_vel=0 does NOT use Quick Stop.
  bool normal_stop_ramp_active_{false};
  double normal_stop_ref_left_{0.0};
  double normal_stop_ref_right_{0.0};

  // 6064h Position actual value [count]
  int32_t raw_position_left_{0};
  int32_t raw_position_right_{0};
  int32_t last_raw_position_left_{0};
  int32_t last_raw_position_right_{0};

  // Unwrapped encoder count from the first received 6064h sample.
  int64_t accumulated_counts_left_{0};
  int64_t accumulated_counts_right_{0};
  bool position_initialized_{false};

  // Unit conversion
  // rad/s -> 0.1 rpm
  static constexpr double RAD_PER_SEC_TO_01RPM = 95.4929658551;

  // 0.1 rpm -> rad/s
  static constexpr double RPM01_TO_RAD_PER_SEC = 0.01047197551;

  // ZLLG45ASM200 encoder setting is 1024 lines.
  // The 6064h feedback is treated as 4x quadrature count:
  // 1024 * 4 = 4096 counts / wheel revolution.
  static constexpr double ENCODER_COUNTS_PER_REV = 4096.0;
  static constexpr double TWO_PI = 6.28318530717958647692;
  static constexpr double RAD_PER_ENCODER_COUNT =
    TWO_PI / ENCODER_COUNTS_PER_REV;
};

}  // namespace zlac8015d_hardware

#endif  // ZLAC8015D_SYSTEM_HPP_

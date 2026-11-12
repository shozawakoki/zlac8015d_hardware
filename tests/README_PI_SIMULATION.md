# PI three-stage transition: offline virtual-wheel model

**No CAN, ROS 2, or motors.** This study uses the same production
`pi_transition_lifecycle.hpp` for transition assistance and an explicitly
simplified PI/BREAKAWAY/stop surrogate. It **does not** execute the full
`Zlac8015dSystemHardware::compute_current_command()` state machine.
Consequently, these results cannot certify real-robot control safety.

## Build and run

```bash
cd ~/ros2_ws/src/zlac8015d_hardware
git pull --ff-only
mkdir -p /tmp/pi_three_stage_sim
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Iinclude \
  tests/simulate_pi_three_stage.cpp -o /tmp/simulate_pi_three_stage
/tmp/simulate_pi_three_stage /tmp/pi_three_stage_sim | \
  tee /tmp/pi_three_stage_sim/summary.csv
python3 tests/check_simulation_outputs.py /tmp/pi_three_stage_sim
```

## Assumptions and limits

- Fixed step: 0.005 s; 5-second total (0-1 s standstill, 1-3 s
  0.05 m/s wheel-speed demand, 3-5 s zero-speed demand).
- Nominal motor/current model: wheel-speed acceleration gain `b0=18
  rad/s²/A`, transport delay `L=0.023 s`, approximated as **0.025 s**
  by 5-step FIFO at this integration interval.
- Coulomb/static friction threshold (illustrative): `0`, `0.2`,
  `0.4 A`; **none of these values are identified from robot data**.
  For zero friction the model corresponds to the identified
  `18/s * exp(-0.023s)` up to delay discretization.
- Single virtual wheel; both real wheels, coupled chassis dynamics,
  road slip, driver current loop, and measurement quantization
  are absent.
- PI: `Kp=0.4`, `Ki=0.2`, `Kd=0`.
- BREAKAWAY: start at `0.65 A`, increment `0.10 A` every
  20 steps up to `2 A`, entering TRACK after six consecutive
  readings above `0.30 rad/s`.
- Three-stage: start with zero PI integral, independent fading assistance,
  0.10 s maximum, removed at 80% of target speed. Old bumpless seeds
  PI integral to match final BREAKAWAY current.
- Current cap `4 A`, slew-rate cap `20 A/s` while tracking.
  The simple stop law is `I=-0.9*speed` with slew limiting,
  **not** the actual hardware normal-stop ramp.
- `overspeed_threshold_crossed` reports whether a simulated speed exceeded
  1.44 rad/s (the prior 1.2 rad/s monitor threshold multiplied by 1.2).
  The simulator continues running after crossing this threshold so traces
  can be inspected; it does **not** simulate a physical safety stop.

## Interpretation

The output contains 9 per-case CSV traces and a summary CSV. Compare:

1. Transition timing relative to speed approaching the 80% threshold.
2. Whether large PI integral persists after the legacy bumpless transition.
3. Peak speed, RMSE over the active 1-3 s segment and current command;
   a lower RMSE alone is insufficient if speed safety limits are violated.
4. Decay after the stop request, which is only informative for the surrogate
   stop law.

Do **not** select physical-test current limits or emergency-stop timing from
this simplified plant. The next milestone is an integration-capable simulated
hardware plugin that runs the actual controller's PI/stop state machine, plus
verified CAN-loss safety behavior before any motor motion.

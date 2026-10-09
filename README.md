## Milestone 1 — Topic discovery & Live Plotting
**What I did:**
- Launched the base simulation via `ros2 launch bicycle_sim bicycle_sim.launch.py`.
- Inspected the active ROS 2 computational graph using CLI commands (`ros2 node list`, `ros2 topic list`, `ros2 topic info`, `ros2 interface show`).
- Verified the structure of key message types: `/state` (`nav_msgs/msg/Odometry`), `/throttle` & `/steer` (`std_msgs/msg/Float32`), and manual teleop commands (`geometry_msgs/msg/Twist`).
- Configured and validated real-time signal plotting via `rqt_plot` and `PlotJuggler` monitoring `/telemetry/cte/data` and `/telemetry/speed/data`.

**Key findings:**
- **Active Nodes:** `/kinematic_bicycle`, `/lap_analyzer`, `/path_gen`, `/robot_state_publisher`, `/rviz2`.
- **Actuator Topics:**
  - `/throttle` (`std_msgs/msg/Float32`): normalized throttle/braking in [-1.0, 1.0].
  - `/steer` (`std_msgs/msg/Float32`): front wheel angle in radians (positive values turn left).
- **State Feedback:** `/state` (`nav_msgs/msg/Odometry`) broadcasting rear-axle position coordinates (x, y) and velocity (v).
- **Telemetry Channels:** `/telemetry/cte`, `/telemetry/speed`, `/telemetry/heading_err_deg`, and `/telemetry/lap_time`.
- **Initial Baseline State:** Signals are static at zero since the vehicle physics integration has not yet been implemented in the bicycle model.

**Screenshots:**
- RViz base layout: `docs/images/m1_rviz_base.png`
- Live rqt_plot: `docs/images/m1_rqt_plot.png`
- PlotJuggler interface: `docs/images/m1_plotjuggler.png`

**Problems & fixes:**
- `rqt-plot` package name resolution error on Ubuntu 22.04: Fixed by installing `ros-humble-rqt-plot` and `ros-humble-plotjuggler-ros` via apt.
- Signals not visible in `rqt_plot`: Expanded window geometry and explicitly subscribed to the inner `.data` field (`/telemetry/cte/data` and `/telemetry/speed/data`).

**Open questions:** None. Ready to proceed to kinematic model equations.



## Milestone 2 — Vehicle Kinematics and Powertrain Resistance

**What I did:**
- Implemented continuous-time state derivatives in bicycle_sim/bicycle_sim/bicycle_model.py using the rear-axle extended kinematic bicycle formulation.
- Added powertrain longitudinal dynamics modeling forward motor acceleration, aerodynamic drag, and rolling resistance.
- Integrated system states using Forward Euler method with discrete time step dt = 0.1 s.
- Enforced physical boundary conditions: heading angle wrapping within [-pi, pi] and speed clamping within [0, max_speed].
- Verified actuator responses via direct /throttle and /steer commands and executed package unit tests.

**Equations:**
- Kinematics:
  - x_dot = v * cos(theta)
  - y_dot = v * sin(theta)
  - theta_dot = (v / L) * tan(delta)
- Longitudinal Acceleration:
  - v_dot = (k_a * u_throttle) - (c_drag * v^2) - (c_roll * v)
- Forward Euler Numerical Integration:
  - x[k+1] = x[k] + x_dot * dt
  - y[k+1] = y[k] + y_dot * dt
  - theta[k+1] = wrap(theta[k] + theta_dot * dt, [-pi, pi])
  - v[k+1] = clamp(v[k] + v_dot * dt, 0.0, max_speed)

**Implementation notes:**
- Set wheelbase length L = 1.25 m and motor gain k_a = 4.0 m/s^2.
- Braking commands (u_throttle < 0) decelerate the vehicle but are clamped at 0.0 m/s to prevent reverse movement from a standstill.

**Screenshots:**
- terminal output: `docs/images/m2_actuator_test.png`

**Test results:**
- colcon test --packages-select bicycle_sim: Summary: 7 tests, 0 errors, 0 failures, 1 skipped.

**Problems & fixes:** None. Actuator testing confirmed proper forward acceleration and left/right steering deflection.


## Milestone 3 — Teleoperation Bridge and Safety Watchdog

**What I did:**
- Implemented open-loop teleoperation translation in bicycle_control/bicycle_control/teleop_bridge.py.
- Mapped user geometry_msgs/msg/Twist commands from /cmd_vel to normalized /throttle (range -1.0 to 1.0) and steering angle /steer in radians.
- Built a 10 Hz safety watchdog mechanism checking elapsed time since the last velocity message, automatically zeroing commands after a 0.5 s timeout.
- Verified manual driving and steering response using teleop_twist_keyboard alongside live visualization in RViz.

**Formulas & Logic:**
- Linear Velocity to Throttle:
  - u_throttle = clamp(v_cmd / v_max, -1.0, 1.0)
- Angular Velocity to Steering Angle:
  - delta = clamp((omega_cmd / omega_max) * delta_max, -delta_max, delta_max)
- Watchdog Condition:
  - if (t_now - t_last_cmd) > 0.5 s -> u_throttle = 0.0, delta = 0.0

**Screenshots:**
- RViz Manual Teleoperation Run: docs/images/m3_teleop_rviz.png

**Problems & fixes:** None. Vehicle stops gracefully when keyboard commands cease due to the auto-zero watchdog.

## Milestone 4 — Longitudinal Cruise Control (PID Speed Controller)

**What I did:**
- Implemented a discrete-time PID longitudinal controller in `bicycle_control/bicycle_control/longitudinal_pid.py` to regulate vehicle forward speed.
- Incorporated anti-windup clamping on the integrator state to prevent severe speed overshoots caused by drag and rolling resistance latency.
- Clamped actuator output within the valid throttle and brake limits [-1.0, 1.0].
- Integrated the speed regulator into `teleop_bridge.py` under the closed-loop cruise control mode (`use_cruise_control:=true`), subscribing to forward velocity from `/state`.
- Validated step response behavior using PlotJuggler and captured speed tracking performance under a 5.0 m/s command.

**Formulas & Logic:**
- Speed Error:
  - e_v = v_target - v_current
- Proportional Term:
  - P = Kp * e_v
- Integral Term with Anti-Windup Clamping:
  - integral = clamp(integral + e_v * dt, -integral_limit, integral_limit)
  - I = Ki * integral
- Derivative Term:
  - D = Kd * ((e_v - prev_error) / dt)
- Control Output:
  - u_raw = P + I + D
  - u_throttle = clamp(u_raw, -max_brake, max_throttle)

**Screenshots:**
- Cruise Control Step Response & RViz Run: `docs/images/m4_cruise_speed_plot1.png` , `docs/images/m4_cruise_speed_plot2.png`

**Problems & fixes:**
- Problem: Integrator could drift and wind up when accelerating against aerodynamic drag.
- Fix: Bounded integrator accumulation between -2.0 and 2.0 via `np.clip`.


## Milestone 5 – Autonomous Trajectory Tracking & Closed-Loop Control

**What I did:**
- Implemented a complete autonomous tracking suite comprising curvature-limited velocity profiling, lateral PID, adaptive Pure Pursuit, and kinematic MPC.
- Formulated path projection logic onto Frenet coordinates to evaluate real-time orthogonal Cross-Track Error (CTE) and heading error relative to centerline waypoints.
- Built `lap_analyzer.py` as an independent observer node broadcasting plottable telemetry signals, dynamic RViz error whiskers, and a floating 3D HUD scoreboard.
- Benchmarked all controllers across full laps on the 528.20 m circuit to evaluate tracking precision, lap timing, and actuator stability.

**Formulas & Logic:**
- Curvature-Constrained Speed Profiling:
  - kappa = 2 * sin(delta_psi) / chord_length
  - v_ref = clamp(sqrt(a_lat_max / abs(kappa)), v_min, v_max)
- Lateral PID Steering Law:
  - delta = - clamp(Kp * e_lat + Ki * integral(e_lat) + Kd * derivative(e_lat) + Kyaw * e_yaw, -max_steer, max_steer)
- Adaptive Pure Pursuit Geometry:
  - Ld = clamp(kv * v + l_min, l_min, l_max)
  - y_local = -sin(yaw) * (x_target - x) + cos(yaw) * (y_target - y)
  - delta = clamp(atan(2 * L * y_local / (Ld^2)), -max_steer, max_steer)
- Kinematic MPC Optimization:
  - State: x = [x, y, yaw, v], Controls: u = [delta_k, a_k]
  - Cost: J = sum(w_lat * e_lat^2 + w_yaw * e_yaw^2 + w_v * e_v^2 + w_dsteer * delta_rate^2 + w_accel * a^2)
  - Warm-Start Shift: u_init = [u_1, u_2, ..., u_N-1, u_N-1] shifted forward by 2 indices per time step.

**Screenshots:**
- Lateral PID Lap Tracking: docs/images/m5_pid_lap.png
- Lateral PID Telemetry & Lap Stats: docs/images/m5_pid_lap_stats.png
- Pure Pursuit Lap Tracking: docs/images/m5_pure_pursuit_lap.png
- Pure Pursuit Telemetry & Lap Stats: docs/images/m5_pure_pursuit_lap_stats.png
- Kinematic MPC Lap Tracking: docs/images/m5_mpc_lap.png
- Kinematic MPC Telemetry & Lap Stats: docs/images/m5_mpc_lap_stats.png

**Problems & fixes:** 
- Excessive integral windup on hairpin exits during lateral PID testing was resolved by implementing an error deadband window around the integrator.
- SLSQP solver latency in MPC was eliminated by feeding a 2-index shifted warm-start vector from the previous solution horizon.


## Milestone 6 – Synthesis & Free Exploration

**What I did:**
- Explored the extension of 2D planar kinematic bicycle models into 4-wheel Ackermann multi-body kinematics, 3D physics engines (Gazebo / MVSim), and stochastic predictive controllers (Nav2 MPPI).

**Key Takeaways:**
- 4-Wheel Ackermann Kinematics: In physical vehicles, inner and outer wheels follow different turning radii (R - W/2 vs R + W/2) during cornering; Ackermann geometry adjusts steering angles dynamically to prevent tire scrub and tread wear.
- 2D vs 3D Simulation: 2D planar models assume infinite grip and zero tire sideslip, making them fast for real-time control; 3D simulators model Pacejka tire curves, suspension travel, and dynamic load transfers.
- Deterministic MPC vs Nav2 MPPI: SciPy SLSQP MPC solves an analytical gradient problem over smooth constraints; MPPI generates thousands of GPU-parallelized Monte Carlo rollouts, handling non-differentiable cost maps and obstacle barriers without gradients.


## Milestone 7 – Controller Benchmarking & Leaderboard

**What I did:**
- Benchmarked all three steering control algorithms (Lateral PID, Adaptive Pure Pursuit, and Kinematic MPC) on the full 528.20 m centerline track.
- Captured full lap metrics through the custom lap analyzer node, logging best lap time, peak velocity, mean cross-track error, maximum dynamic deviation, and RMS tracking error.
- Verified tracking telemetry live through RViz 3D HUD markers and plotted error whiskers.

**Leaderboard Results:**
- Pure Pursuit achieved the fastest lap time (68.89 s) with superior geometric tracking (Mean CTE: 0.035 m, Max CTE: 0.367 m, RMS: 0.059 m).
- Lateral PID achieved a 78.86 s lap time with a peak speed of 7.65 m/s, but suffered from significant corner overshoot on hairpins (Max CTE: 3.119 m, RMS: 0.627 m).
- Kinematic MPC achieved highly stable tracking (Mean CTE: 0.074 m, Max CTE: 0.355 m, RMS: 0.099 m), but recorded a slower lap time (121.80 s) due to conservative acceleration optimization and solver computation overhead.

**Screenshots:**
- Lateral PID Lap Stats: docs/images/m5_pid_lap_stats.png
- Pure Pursuit Lap Stats: docs/images/m5_pure_pursuit_lap_stats.png
- Kinematic MPC Lap Stats: docs/images/m5_mpc_lap_stats.png

**Problems & fixes:**
- Live HUD text and lap summary buffers required resetting between laps to prevent accumulating stale CTE data across consecutive runs.

## 🏆 Telemetry Benchmark Leaderboard

| Controller Mode | Best Lap Time (s) | Top Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps Completed / Status |
|---|---|---|---|---|---|---|
| **Lateral PID (Reactive)** | 78.86 | 7.65 | 0.443 | 3.119 | 0.627 | 2 Laps / Fast, Severe Corner Overshoot |
| **Pure Pursuit (Preview)** | 68.89 | 7.66 | 0.035 | 0.367 | 0.059 | 2 Laps / Fastest & High Geometry Accuracy |
| **Extended Kinematic MPC (Optimal)** | 121.80 | 3.94 | 0.074 | 0.355 | 0.099 | 2 Laps / Conservative Throttle, Smooth & Optimal Tracking |

---

## 💡 Synthesis & Architectural Discussion

### 1. Kinematics vs Multi-Body (Ackermann Dynamics)
In a physical four-wheel vehicle navigating a turn, the inner and outer wheels follow concentric circles with different turning radii (R - W/2 vs R + W/2). Steering both wheels at the exact same angle forces lateral tire scrub, accelerating tire wear and degrading tracking performance. Ackermann steering geometry enforces:
- tan(delta_inner) = L / (R - W / 2)
- tan(delta_outer) = L / (R + W / 2)

In production frameworks like `ros2_control`, this multi-body joint relationship is parameterized through kinematic hardware interfaces and multi-joint transmission controllers rather than a single rigid bicycle assumption.

### 2. 2D Planar Simulation vs 3D Multi-Body Engines (Gazebo / MVSim)
Planar 2D kinematic models assume infinite road grip and zero lateral slip (alpha = 0), neglecting body roll, pitch, suspension travel, and dynamic tire load shifts. While computationally minimal and ideal for real-time 10 Hz MPC preview optimization, they cannot capture vehicle behavior near the friction limits. In contrast, 3D physics engines (such as Gazebo or MVSim) simulate Pacejka Magic Formula tire friction curves, suspension compliance, track surface elevations, and sensor noise, capturing understeer, oversteer, and lateral sliding phenomena.

### 3. Deterministic MPC vs Sampling-Based Optimal Control (Nav2 MPPI)
Deterministic gradient-based solvers like SciPy SLSQP minimize trajectory tracking error by calculating numerical gradients over continuous objective functions and actuator limits. They provide mathematically precise tracking and strict constraint satisfaction, but introduce optimization latency (resulting in conservative speeds and longer lap times, as observed in our 121.80 s benchmark). Conversely, Model Predictive Path Integral (MPPI) control leverages massively parallel Monte Carlo rollouts (often GPU-accelerated) to sample thousands of randomized trajectories simultaneously. MPPI naturally accommodates non-differentiable cost maps, obstacles, and discontinuous penalties without gradient evaluations, offering superior robustness for unstructured navigation at the cost of higher raw compute requirements.


## 🎥Milestone 8 - Video Demonstration
- Watch the full walkthrough and simulation demo: https://drive.google.com/drive/folders/1gmZGxQgs4i2aZCs71jfDhLrLD_PSABim?usp=drive_link
  

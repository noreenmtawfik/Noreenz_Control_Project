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
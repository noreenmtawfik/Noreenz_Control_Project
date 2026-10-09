"""
High-Level Lateral Steering Controller: Reactive Lateral PID.
Steers based on instantaneous Cross-Track Error (CTE) and Heading Error.
"""

import math
import numpy as np  # noqa: F401


class LateralPIDController:
    """Lateral PID steering controller based on Cross-Track Error (CTE) and Heading Error.

    Commands front wheel steering based on instantaneous lateral offset (cross-track error)
    and orientation error relative to the nearest path waypoint.
    """

    def __init__(self, kp=0.8, ki=0.02, kd=0.15, k_yaw=0.5, dt=0.1,
                 max_steer_rad=math.radians(35.0), integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.k_yaw = k_yaw
        self.dt = dt
        self.max_steer_rad = max_steer_rad
        self.integral_limit = integral_limit

        self.integral_cte = 0.0
        self.prev_cte = 0.0

    def compute_steering(self, cte, heading_err):
        """Computes front wheel steering angle delta in radians.

        Args:
            cte: Signed cross-track error in meters (positive = vehicle is left of path).
            heading_err: Heading error in radians (psi_vehicle - psi_path).

        Returns:
            delta_rad: Commanded front steering angle in radians [-max_steer_rad, max_steer_rad].
        """
        # TODO: Milestone 5.2 — Reactive Lateral PID Controller
        # This is the lateral steering controller. It corrects for how far the car
        # is off the path (CTE) and how misaligned its heading is.
        # Implement PID on the CTE with anti-windup, add a heading correction term,
        # and clamp the output to the steering limits.
        
        # 1. Update integral with anti-windup clamping
        self.integral_cte += cte * self.dt
        self.integral_cte = float(np.clip(self.integral_cte, -self.integral_limit, self.integral_limit))

        # 2. Derivative of CTE
        d_cte = (cte - self.prev_cte) / self.dt if self.dt > 0.0 else 0.0
        self.prev_cte = cte

        # 3. PID terms on CTE (negative sign to steer back toward path)
        p_term = -self.kp * cte
        i_term = -self.ki * self.integral_cte
        d_term = -self.kd * d_cte

        # 4. Heading error correction term
        yaw_term = -self.k_yaw * heading_err

        # 5. Raw commanded steering angle
        delta = p_term + i_term + d_term + yaw_term

        # 6. Actuator limit clamping [-max_steer_rad, max_steer_rad]
        delta_clamped = float(np.clip(delta, -self.max_steer_rad, self.max_steer_rad))
        return delta_clamped

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral_cte = 0.0
        self.prev_cte = 0.0

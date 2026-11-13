"""Extended Kalman filter of the car relative to its lane (no ROS, unit-tested).

State x = [e_y, psi, v]
  e_y : lateral offset, + = car is LEFT of the lane centre (m)         <- camera `ego_lane_offset`
  psi : car heading minus lane direction, + = pointing left of lane (rad) <- camera -`heading_error`
  v   : forward speed of base_footprint (m/s)                          <- rear-wheel encoder
Predict = kinematic bicycle driven by the measured steering angle; each sensor is a scalar update.
"""
import math

import numpy as np

WHEELBASE = 1.704  # m (enimia.urdf)


class LaneEKF:
    def __init__(self, wheelbase=WHEELBASE, q=(0.02, 0.01, 0.5), gate=9.0, max_rejects=5):
        self.L = wheelbase
        self.max_rejects = max_rejects   # consecutive outliers on one state before it is re-initialised
        self.rejects = [0, 0, 0]
        self.x = np.zeros(3)
        self.P = np.diag([1.0, 0.3, 1.0])
        self.Q = np.diag(q)          # process noise per second (e_y, psi, v)
        self.gate = gate             # innovation^2 / S above this = outlier, update skipped

    def predict(self, dt, steer):
        if dt <= 0:
            return
        e, psi, v = self.x
        F = np.array([[1.0, v * math.cos(psi) * dt, math.sin(psi) * dt],
                      [0.0, 1.0, math.tan(steer) / self.L * dt],
                      [0.0, 0.0, 1.0]])
        self.x = np.array([e + v * math.sin(psi) * dt,
                           psi + v * math.tan(steer) / self.L * dt,
                           v])
        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, index, z, r):
        """Scalar measurement of state[index] with variance r. Returns False if rejected as an outlier."""
        if not math.isfinite(z):
            return False
        H = np.zeros(3)
        H[index] = 1.0
        innov = z - self.x[index]
        S = H @ self.P @ H + r
        if innov * innov / S > self.gate:
            # a few rejects = one-off outlier; many in a row = the filter is wrong (camera jumped to another
            # lane, model drifted), so trust the sensor again instead of rejecting it forever
            self.rejects[index] += 1
            if self.rejects[index] < self.max_rejects:
                return False
            self.x[index] = z
            self.P[index, :] = 0.0
            self.P[:, index] = 0.0
            self.P[index, index] = r
            self.rejects[index] = 0
            return True
        self.rejects[index] = 0
        K = self.P @ H / S
        self.x = self.x + K * innov
        self.P = (np.eye(3) - np.outer(K, H)) @ self.P
        return True

    @property
    def offset(self):
        return float(self.x[0])

    @property
    def heading(self):
        return float(self.x[1])

    @property
    def speed(self):
        return float(self.x[2])

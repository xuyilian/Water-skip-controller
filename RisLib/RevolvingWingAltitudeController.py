import math

class RevolvingWingAltitudeController:
    def __init__(self, lam0, lam1, lam2, a1, a2, a3, a4, a5, a6, max_thrust):
        self.lam0 = lam0
        self.lam1 = lam1
        self.lam2 = lam2
        self.A1 = a1
        self.A2 = a2
        self.A3 = a3
        self.A4 = a4
        self.A5 = a5
        self.A6 = a6
        self.time_delay = 0
        self.T_delay = 0
        self.max_thrust = max_thrust

    def step(self, z, z1, omega, z3_desired, z2_desired, z1_desired, z_desired, time_now):
        dt = time_now - self.time_delay

        sign_omega = math.copysign(1, omega)
        T = -(self.lam1 * (z1 - z1_desired) - z3_desired + self.lam0 * (z - z_desired) - self.lam2 * (
                z2_desired - self.A2 * z1 - self.A1 * omega * abs(omega) + 49 / 5) + self.A2 * (
                      self.A2 * z1 + self.A1 * omega * abs(omega) - 49 / 5) - (
                      self.A3 * self.T_delay) / dt + 2 * self.A1 * omega * sign_omega * (
                      self.A5 * z1 + self.A4 * omega * abs(omega))) / (
                    self.A3 * self.lam2 + self.A3 / dt + self.A2 * self.A3 +
                    2 * self.A1 * self.A6 * omega * sign_omega)

        self.T_delay = T
        self.time_delay = time_now

        thrust = T * self.max_thrust

        if thrust < 0:
            thrust = 0
        elif thrust > self.max_thrust:
            thrust = self.max_thrust
        return thrust
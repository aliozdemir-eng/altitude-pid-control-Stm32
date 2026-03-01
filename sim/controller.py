class PIDController:
    """
    Discrete-time PID controller (embedded-oriented).

    Features:
    - Fixed-step update (Ts)
    - Output saturation (u_min..u_max)
    - Integral clamping (anti-windup)
    - Optional feedforward term (u_ff)
    """

    def __init__(
        self,
        Kp=1.0, Ki=0.0, Kd=0.0,
        u_min=0.0, u_max=1.0,
        I_min=-5.0, I_max=5.0,
        u_ff=0.0
    ):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.u_min = u_min
        self.u_max = u_max
        self.I_min = I_min
        self.I_max = I_max

        self.u_ff = u_ff

        self.I = 0.0
        self.e_prev = 0.0

    def reset(self):
        self.I = 0.0
        self.e_prev = 0.0

    def update(self, setpoint, measurement, Ts):
        e = setpoint - measurement

        # integral update + clamp
        self.I += e * Ts
        if self.I > self.I_max:
            self.I = self.I_max
        elif self.I < self.I_min:
            self.I = self.I_min

        # derivative
        D = (e - self.e_prev) / Ts
        self.e_prev = e

        # raw control
        u = (self.Kp * e) + (self.Ki * self.I) + (self.Kd * D) + self.u_ff

        # saturation
        if u > self.u_max:
            u = self.u_max
        elif u < self.u_min:
            u = self.u_min

        return u, e

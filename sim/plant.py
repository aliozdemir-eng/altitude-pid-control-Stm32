class VerticalPlant:
    def __init__(self, ku=15.0, kd=0.08, g=9.81):
        self.ku = ku
        self.kd = kd
        self.g = g
        self.h = 0.0
        self.v = 0.0

    def update(self, u, Ts):
        # Acceleration model
        a = self.ku * u - self.g - self.kd * self.v * abs(self.v)

        # State update
        self.v += a * Ts
        self.h += self.v * Ts

        return self.h, self.v

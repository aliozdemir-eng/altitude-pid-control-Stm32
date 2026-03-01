import numpy as np
import matplotlib.pyplot as plt

from plant import VerticalPlant
from controller import PIDController
from sensor import AltitudeSensor


def run_simulation():

    # --- Simulation parameters ---
    Ts = 0.01
    T_end = 20
    t = np.arange(0, T_end, Ts)

    # --- Components ---
    plant = VerticalPlant()
    sensor = AltitudeSensor(noise_std=0.02, alpha=0.15)

    pid = PIDController(
        Kp=1.2,
        Ki=0.6,
        Kd=0.05,
        u_min=0.0,
        u_max=1.0,
        I_min=-5,
        I_max=5,
        u_ff=9.81 / 15.0  # gravity compensation feedforward
    )

    # --- Setpoint ---
    h_set = np.zeros_like(t)
    h_set[t >= 2] = 10.0

    # --- Logging ---
    h_log = []
    h_meas_log = []
    u_log = []
    e_log = []

    # --- Main loop ---
    for i in range(len(t)):

        # Sensor measurement
        h_meas = sensor.measure(plant.h)

        # Control update
        u, e = pid.update(h_set[i], h_meas, Ts)

        # Plant update
        h, v = plant.update(u, Ts)

        # Logging
        h_log.append(h)
        h_meas_log.append(h_meas)
        u_log.append(u)
        e_log.append(e)

    # --- Plot ---
    plt.figure(figsize=(10, 8))

    plt.subplot(3, 1, 1)
    plt.plot(t, h_set, '--', label="Setpoint")
    plt.plot(t, h_log, label="True Height")
    plt.plot(t, h_meas_log, label="Measured Height", alpha=0.6)
    plt.ylabel("Altitude (m)")
    plt.legend()
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(t, u_log)
    plt.ylabel("Control Input")
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(t, e_log)
    plt.ylabel("Error")
    plt.xlabel("Time (s)")
    plt.grid()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_simulation()

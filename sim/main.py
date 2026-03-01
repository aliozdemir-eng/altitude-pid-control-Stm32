import os
import csv
import numpy as np
import matplotlib.pyplot as plt

from sim.plant import VerticalPlant
from sim.controller import PIDController
from sim.sensor import AltitudeSensor
from sim.metrics import step_metrics


def run_simulation():
    # -----------------------------
    # Simulation parameters
    # -----------------------------
    Ts = 0.01
    T_end = 20.0
    t = np.arange(0.0, T_end, Ts)

    # -----------------------------
    # Components (Plant / Sensor / Controller)
    # -----------------------------
    plant = VerticalPlant(ku=15.0, kd=0.08, g=9.81)

    sensor = AltitudeSensor(
        noise_std=0.02,   # measurement noise std
        alpha=0.15        # LPF alpha (smaller => smoother)
    )

    pid = PIDController(
        Kp=1.2,
        Ki=0.6,
        Kd=0.05,
        u_min=0.0,
        u_max=1.0,
        I_min=-5.0,
        I_max=5.0,
        u_ff=plant.g / plant.ku  # gravity feedforward
    )

    # -----------------------------
    # Setpoint (step at t = 2s)
    # -----------------------------
    h_set = np.zeros_like(t)
    h_set[t >= 2.0] = 10.0

    # -----------------------------
    # Logging
    # -----------------------------
    h_log = []
    h_meas_log = []
    u_log = []
    e_log = []

    # -----------------------------
    # Main discrete-time loop
    # -----------------------------
    for i in range(len(t)):
        # Sensor measurement (noise + LPF)
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

    # Convert lists to arrays
    h_log = np.array(h_log)
    h_meas_log = np.array(h_meas_log)
    u_log = np.array(u_log)
    e_log = np.array(e_log)

    # -----------------------------
    # Performance Metrics
    # -----------------------------
    results = step_metrics(t, h_set, h_log, step_time=2.0, band_percent=2.0)

    print("\n--- Step Response Performance ---")
    print(f"Overshoot: {results['overshoot']:.2f} %")
    print(f"Settling Time: {results['settling_time']:.2f} s")
    print(f"Steady-State Error: {results['sse']:.4f} m")

    # -----------------------------
    # Export results
    # -----------------------------
    os.makedirs("results", exist_ok=True)

    # 1) metrics.txt
    with open("results/metrics.txt", "w", encoding="utf-8") as f:
        f.write("--- Step Response Performance ---\n")
        f.write(f"Overshoot: {results['overshoot']:.2f} %\n")
        f.write(f"Settling Time: {results['settling_time']:.2f} s\n")
        f.write(f"Steady-State Error: {results['sse']:.4f} m\n")

    # 2) telemetry.csv
    with open("results/telemetry.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "h_set", "h_true", "h_meas", "u", "error"])
        for i in range(len(t)):
            writer.writerow([t[i], h_set[i], h_log[i], h_meas_log[i], u_log[i], e_log[i]])

    # -----------------------------
    # Plot (saved to PNG)
    # -----------------------------
    plt.figure(figsize=(10, 8))

    plt.subplot(3, 1, 1)
    plt.plot(t, h_set, "--", label="Setpoint")
    plt.plot(t, h_log, label="True Height")
    plt.plot(t, h_meas_log, label="Measured Height", alpha=0.6)
    plt.ylabel("Altitude (m)")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(t, u_log)
    plt.ylabel("Control Input (u)")
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(t, e_log)
    plt.ylabel("Error (e)")
    plt.xlabel("Time (s)")
    plt.grid(True)

    plt.tight_layout()
    plt.savefig("results/closed_loop_response.png", dpi=200)
    plt.close()


if __name__ == "__main__":
    run_simulation()

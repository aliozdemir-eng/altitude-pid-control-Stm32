import numpy as np
import matplotlib.pyplot as plt
import os

# Simulation parameters
Ts = 0.01      # 10 ms sampling
T_end = 20
t = np.arange(0, T_end, Ts)

# Toy vertical dynamics parameters
g = 9.81
ku = 15.0
kd = 0.08

# Setpoint
h_set = np.zeros_like(t)
h_set[t >= 2] = 10

# PID parameters
Kp = 1.2
Ki = 0.6
Kd = 0.0

I_min = -5
I_max = 5

# States
h = 0
v = 0

# Logging arrays
h_log = []
u_log = []
e_log = []

I = 0
e_prev = 0

for i in range(len(t)):
    e = h_set[i] - h
    
    I += e * Ts
    I = max(min(I, I_max), I_min)
    
    D = (e - e_prev) / Ts
    u = Kp*e + Ki*I + Kd*D + g/ku
    u = max(min(u, 1), 0)
    
    e_prev = e
    
    a = ku*u - g - kd*v*abs(v)
    v += a * Ts
    h += v * Ts
    
    h_log.append(h)
    u_log.append(u)
    e_log.append(e)

# Convert to numpy
h_log = np.array(h_log)
u_log = np.array(u_log)
e_log = np.array(e_log)

# Create results folder
os.makedirs("results", exist_ok=True)

# Plot altitude
plt.figure()
plt.plot(t, h_set, label="Setpoint")
plt.plot(t, h_log, label="Altitude")
plt.legend()
plt.grid()
plt.title("Altitude Tracking")
plt.savefig("results/altitude_tracking.png")

# Plot control
plt.figure()
plt.plot(t, u_log)
plt.grid()
plt.title("Control Effort")
plt.savefig("results/control_effort.png")

print("Simulation code added successfully.")

# Real-Time Altitude Control System (PID-Based)  
### Embedded-Oriented Flight Dynamics Simulation (STM32-Ready)

---

## 1. Project Objective

This project implements a closed-loop altitude control system using a PID controller on a simplified vertical flight dynamics model.  

The primary goal is to simulate a real-time control architecture that can be directly ported to an embedded platform (STM32) for hardware implementation.

The system models:

- Vertical motion dynamics (altitude + velocity states)
- Gravity compensation
- Nonlinear aerodynamic drag
- Actuator saturation
- Integral anti-windup protection

The project follows an embedded-oriented design philosophy: fixed sampling time, deterministic update loop, and modular structure.

---

## 2. System Modeling

### 2.1 State Variables

The system is modeled using two primary states:

- h(t) → altitude
- v(t) → vertical velocity

### 2.2 Continuous-Time Model (Simplified)

a = ku·u − g − kd·v|v|

Where:

- u → normalized thrust command (0–1)
- ku → control effectiveness coefficient
- g → gravitational acceleration
- kd → nonlinear drag coefficient

Discrete-time implementation (Ts = 10 ms):

v(k+1) = v(k) + a(k)Ts  
h(k+1) = h(k) + v(k+1)Ts  

---

## 3. Control Architecture

Closed-loop structure:

Altitude Setpoint → Error Computation → PID Controller → Thrust Command → Vehicle Dynamics → Feedback

### 3.1 PID Controller

u(k) = Kp·e(k) + Ki·∫e(k)dt + Kd·de/dt + u_ff

Features implemented:

- Integral clamping (anti-windup)
- Actuator saturation (0 ≤ u ≤ 1)
- Gravity feedforward compensation
- Fixed-step sampling (Ts = 10 ms)

This structure mirrors typical embedded flight-control implementations.

---

## 4. Embedded-Oriented Design Principles

Although currently implemented in Python for rapid prototyping, the architecture is intentionally designed for embedded deployment:

- Deterministic control loop
- Explicit state update
- No dynamic memory allocation
- Fixed sampling period
- Modular structure (control / plant separation)

This allows direct migration to:

- STM32 (Timer ISR at 100 Hz)
- UART telemetry streaming
- Real-time logging and performance evaluation

---

## 5. Performance Considerations

The simulation allows evaluation of:

- Rise time
- Overshoot
- Settling time
- Steady-state error
- Control effort saturation behavior

The structure is suitable for further extension with:

- Sensor noise modeling
- Low-pass filtering
- Kalman filtering
- State-space control (LQR)
- Hardware-in-the-loop (HIL) testing

---

## 6. Technologies Used

- Python
- NumPy
- Matplotlib
- Discrete-Time Control Systems
- Embedded-Oriented Control Design

---

## 7. Future Extensions

- Add sensor noise + filtering
- Implement full telemetry export (CSV logging)
- Compare PID vs LQR
- Implement state observer
- Port to STM32 (HAL + Timer ISR + UART logging)
- Real-time visualization dashboard

---

## 8. Application Domains

This project demonstrates foundational principles applicable to:

- Flight Control Systems
- UAV stabilization
- Rocket altitude control simulations
- Embedded real-time control
- Aerospace control prototyping

---
---

## How to Run (Local / Codespaces)

> Repository is structured as a Python package (`sim/`).

```bash
pip install numpy matplotlib
python -m sim.main
```

---

## Repository Structure

```text
sim/
 ├── __init__.py      # package marker
 ├── main.py          # entry point (simulation runner)
 ├── plant.py         # nonlinear vertical dynamics model
 ├── controller.py    # discrete-time PID (anti-windup, saturation, feedforward)
 ├── sensor.py        # noise + low-pass filtering sensor model
 └── metrics.py       # step response metrics (overshoot, settling time, SSE)
```

---

## Key Engineering Features

- Fixed-step discrete-time loop (Ts = 10 ms)
- Nonlinear vertical plant model (gravity + aerodynamic drag)
- Actuator saturation limits
- Integral anti-windup protection
- Gravity feedforward compensation
- Sensor noise modeling (Gaussian)
- First-order low-pass filtering
- Step response performance metrics (Overshoot, Settling Time, SSE)

---

## Performance Evaluation

The simulation automatically computes:

- **Overshoot (%)**
- **Settling Time (2% band)**
- **Steady-State Error (m)**

Metrics are calculated in `sim/metrics.py` and printed to console during simulation.

## Keywords

Closed-Loop Control  
PID Tuning  
Discrete-Time Control  
Embedded Systems  
Flight Dynamics  
Real-Time Systems  
Anti-Windup  
STM32-Oriented Design  
Control Engineering

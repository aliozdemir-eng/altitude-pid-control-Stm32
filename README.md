# Real-Time Altitude Control with PID (STM32-Oriented Simulation)

This project demonstrates a closed-loop altitude control system using a PID controller.

## 🔹 Project Overview
- Vertical motion toy model (altitude + velocity)
- Closed-loop PID control
- Gravity compensation
- Actuator saturation
- Anti-windup protection

The goal of this project is to simulate a flight-control-style altitude stabilization system that can later be ported to STM32 for real-time implementation.

---

## 🔹 Control Structure

Altitude error → PID → Thrust control → Vehicle dynamics → Feedback

---

## 🔹 Technologies
- Python
- Control Systems (PID)
- Embedded-ready architecture
- Real-time sampling (10 ms)

---

## 🔹 Future Work
- Add sensor noise model
- Add filtering (low-pass / Kalman)
- Export telemetry (CSV)
- Port to STM32 (Timer ISR + UART logging)

---

## 🔹 Keywords
Closed-Loop Control, PID Tuning, Real-Time Systems, Flight Control Simulation, Embedded Systems

# OmniBotCC — Robotat Omnidirectional Agent

**Overview**  
OmniBotCC is a complete project (firmware, MATLAB utilities and Python client) for an **omnidirectional mobile platform** designed to operate inside the **Robotat** ecosystem at the *Universidad del Valle de Guatemala*. The repository provides a reproducible toolset for control experiments, trajectory planning and tracking, simulation, and teaching.

---

## Repository layout (high-level)

- `ESP32/` — ESP32 firmware (ESP-IDF): closed-loop control of 4 stepper motors, AS5600 encoder readings (ADC), per-motor PID and UDP communication.  
- `Funciones_Matlab/` — MATLAB scripts and functions for trajectory planning, Robotat connection, simulation, visualization, calibration and data analysis.  
- `Funciones_Python/` — Python tools and example scripts for sending UDP commands to the ESP32 and quick demos.  
---

## Key features

- Closed-loop PID speed control for four stepper motors (one PID per motor).  
- AS5600 magnetic encoder feedback using ADC for angle and speed estimation.  
- UDP-based command & telemetry over Wi-Fi (integrates with Robotat server).  
- Timer-based step generation with configurable microstepping (example: up to 1/32).  
- FreeRTOS-based multitasking (encoder sampling ISR, PID task, motor control task).  
- MATLAB toolset for trajectory generation, Robotat integration, calibration and post-processing.  
- Python client for quick demos and automation.

---

## System architecture (summary)

**Core components**

- **ESP32 MCU** — main controller: RTOS tasks, PID loops, UDP server.  
- **Stepper drivers** — accept STEP/DIR signals per wheel.  
- **AS5600 encoders** — analog outputs read by ADC for angular position.  
- **PC (MATLAB / Python)** — trajectory planning, visualization and commanding.

**Control loop (stepper motors)**

1. Encoder sampling via timer ISR (example: every 50 ms → 20 Hz).  
2. Compute angular difference → estimate RPM per motor.  
3. PID task computes correction (setpoint vs measured RPM).  
4. Motor control task updates timer parameters and generates STEP pulses.  
5. UDP interface accepts textual commands to change setpoints and states in real time.

### Control loop (Robot)

The closed-loop control for the omnidirectional robot is executed in MATLAB, using real-time feedback from **Robotat** to track the platform’s position and orientation while commanding velocities via UDP.  
The process runs in a control loop with sampling time `dt` (typically 50–100 ms) and includes the following stages:

1. **Pose acquisition (feedback):**  
   The current robot pose (`x_real`, `y_real`, `θ_real`) is obtained from the Robotat system using the function `robotat_get_pose()`.  
   The data is corrected by a known offset and converted to meters and radians.

2. **Goal selection (lookahead target):**  
   The controller selects the next goal point along the trajectory using a *lookahead distance* criterion.  
   This ensures smooth progression between waypoints and avoids oscillations when the robot is close to intermediate targets.

3. **Error computation:**  
   The position and orientation errors are calculated as:  
   \[
   e = \begin{bmatrix} x_g - x \\ y_g - y \\ \theta_g - \theta \end{bmatrix}
   \]
   where \( (x_g, y_g, \theta_g) \) is the current goal pose.  
   These errors drive the PI controller and determine the corrective velocities.

4. **Feedforward + PI control law:**  
   The desired linear and angular velocities are estimated from consecutive points in the trajectory (`v_des`, `w_des`).  
   A **Feedforward + PI** scheme computes the commanded velocity vector in the world frame:
   \[
   \dot{q}_{world} = \dot{q}_{d,world} + K_p e + K_i \int e\,dt
   \]
   where `Kp` and `Ki` are tuned empirically.  
   Integrator limits prevent windup (`I_min`, `I_max`).

5. **Transformation to body frame:**  
   The commanded velocities are converted from the world frame to the robot’s local frame using the current orientation `θ`:
   \[
   v_{body} = R(θ)^T \begin{bmatrix} v_x \\ v_y \end{bmatrix}
   \]
   forming the body velocity vector:
   \[
   V_b = \begin{bmatrix} ω \\ v_{bx} \\ v_{by} \end{bmatrix}
   \]

6. **Wheel speed conversion and saturation:**  
   Wheel angular velocities are obtained via:
   \[
   ω_{wheels} = \frac{1}{r} H_0 V_b
   \]
   where `r` is the wheel radius and `H₀` the kinematic matrix of the omnidirectional platform.  
   These are converted to RPM and limited by `rpm_max`.  
   If saturation occurs, the integrator is frozen (anti-windup).

7. **Command transmission (actuation):**  
   The computed wheel RPMs are sent to the physical robot via the UDP interface using:
   ```matlab
   robot_set_velocities(u_rpm);


---

## Example UDP commands

- `start` — enable all motors  
- `stop` — disable all motors  
- `m0_rpm=250` — set motor 0 target to 250 RPM  
- `m1_dir=CW` / `m1_dir=CCW` — set motor 1 direction  
- `m2_start` / `m3_stop` — control individual motors

---

## Default hardware configuration (example)

**Pin mapping (example)**

| Component | GPIOs | Notes |
|---|---:|---|
| Enable (drivers) | 23 | Shared enable for all drivers |
| STEP (m0..m3) | 26, 19, 16, 14 | Step pulses for motors 0–3 |
| DIR (m0..m3) | 25, 18, 4, 27 | Direction pins for motors 0–3 |
| AS5600 ADC inputs | 34, 35, 32, 33 | Analog encoder inputs |
| Wi-Fi status LED | 2 | Connection status indicator |

**Motor parameters (examples)**

- Steps per revolution: 200  
- Microstepping: 1/32 (configurable)  
- Approx. maximum RPM: ~400  
- Example PID starting gains (tune on hardware): `Kp = 0.04, Ki = 0.002, Kd = 0.0002`
---

## Build & flash (firmware)

1. Install ESP-IDF toolchain (recommended: ESP-IDF v5.x). Follow Espressif's official instructions.  
2. Clone this repository.  
3. From the `firmware/` folder run:

```bash
idf.py set-target esp32
idf.py build
idf.py flash
idf.py monitor
```

---

## MATLAB overview

The `matlab/` folder contains:

- Robotat connection utilities (`robotat_connect`, `robotat_get_pose`, etc.).
- Calibration and quick-start scripts (`Calibracion_Robot.m`, `Inicio_Rapido.m`).  
- Trajectory generators and visualization scripts (`Trayectoria_Omnidireccional.m`, `Trayectorias_Mapa.m`).  
- Utilities for pose conversions and data logging.

Use `Inicio_Rapido.m` to start experiments and `Trayectoria_Omnidireccional.m` for trajectory execution and analysis.

---

## Python client

The `python/` folder includes:

- `Inicio_Rapido.py` — demo routines with connect, set velocities, start/stop and example motion sequences.  
- `udp_robot.py` — helper functions: `robot_connect`, `robot_disconnect`, `robot_start`, `robot_stop`, `robot_set_velocities`.  

Requirements: Python 3.8+, `numpy` (see `requirements.txt`).

---

## License & attribution

This project is property of **Universidad del Valle de Guatemala**. Contact them for licensing and distribution permissions.

---

## Author

**Author:** Christian Campos  
**Institution:** Universidad del Valle de Guatemala

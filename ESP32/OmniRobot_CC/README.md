# Robotat Omnidirectional Agent (OmniBotCC)

## Overview
**OmniBotCC** is an embedded control application developed using the **ESP-IDF framework** for the **ESP32 microcontroller**. It serves as the main controller for an **omnidirectional robotic agent** within the **Robotat ecosystem** at *Universidad del Valle de Guatemala*.

The system controls **four stepper motors** arranged in an omnidirectional configuration, allowing movement across a 2D plane. Each motor is equipped with an **AS5600 magnetic encoder** for closed-loop feedback, enabling precise **PID speed control**. Communication with the Robotat environment is established via **UDP over Wi-Fi**, allowing external systems (e.g., Robotat server) to send commands and receive updates.

---

## Main Features
- Closed-loop control of four stepper motors using PID.
- High-resolution feedback from AS5600 encoders (ADC-based angle measurement).
- UDP communication for remote control and data exchange.
- Timer-based PWM generation for stepper control (microstepping up to 1/32).
- Wi-Fi initialization with optional LED status indicator.
- Multi-tasking architecture using FreeRTOS:
  - Timer-based periodic callback (20 Hz) for encoder sampling.
  - PID task for closed-loop speed regulation.
  - Motor control task for stepper pulse generation.
- Dynamic command parsing via UDP or UART (optional).
- Robust error handling and logging through ESP-IDF’s `esp_log`.

---

## System Architecture

### Core Components
| Component | Description |
|------------|-------------|
| ESP32 MCU | Central controller executing FreeRTOS tasks and PID loops. |
| Stepper Drivers | Receive STEP/DIR signals for motion control. |
| AS5600 Encoders | Provide analog angular position feedback for speed estimation. |
| Wi-Fi UDP Server | Handles incoming motion and configuration commands. |
| PID Controller | Regulates motor speed using feedback from encoders. |

---

## Control Loop Description

1. **Encoder Sampling (Timer ISR)**  
   The system periodically reads angular positions from the AS5600 sensors every **50 ms (20 Hz)**.  
   Angular differences are converted into estimated RPM values per motor.

2. **PID Task**  
   The PID controller compares desired RPM vs. measured RPM to compute a control signal.  
   The signal adjusts the timer parameters that generate motor steps.

3. **UDP Command Interface**  
   Commands are received as strings through UDP packets. Examples include:
start
stop
m0_rpm=200
m1_dir=CCW
m2_start
m3_stop

Commands dynamically update the control parameters in real time.

4. **Motor Control**  
Each motor runs in closed-loop mode, maintaining its target speed with high precision.

---

## Command Examples

| Command | Description |
|----------|-------------|
| `start` | Enables all motors. |
| `stop` | Disables all motors. |
| `m0_rpm=250` | Sets motor 0 to 250 RPM. |
| `m1_dir=CW` / `m1_dir=CCW` | Sets direction of motor 1. |
| `m2_start` | Activates motor 2 only. |
| `m3_stop` | Stops motor 3 only. |

---

## File Structure

```
OmniRobot_CC/
├─ platformio.ini
├─ README.md
├─ docs/
│  ├─ overview.md
│  └─ wiring.md
├─ firmware/
│  ├─ src/
│  │  ├─ main.cpp
│  │  ├─ MotorController.cpp
│  │  ├─ MotorController.h
│  │  ├─ PID.cpp
│  │  ├─ PID.h
│  │  ├─ AS5600.cpp
│  │  ├─ AS5600.h
│  │  ├─ WiFiManager.cpp
│  │  ├─ WiFiManager.h
│  │  ├─ WebServer.cpp
│  │  └─ config.h
│  ├─ lib/
│  └─ include/
├─ hardware/
│  ├─ schematics/
│  └─ pcb/
├─ tools/
│  ├─ calibrate.py
│  └─ bench_tests/
├─ data/
│  └─ spiffs/
├─ examples/
└─ releases/
    └─ firmware_v1.0.bin
```


---

## Hardware Configuration

### Pinout Summary
| Component | GPIOs | Description |
|------------|-------|-------------|
| Enable | 23 | Shared enable pin for all drivers |
| Stepper STEP Pins | 26, 19, 16, 14 | Pulse control for motors 0–3 |
| Stepper DIR Pins | 25, 18, 4, 27 | Direction control for motors 0–3 |
| AS5600 Inputs | 34, 35, 32, 33 | ADC inputs for encoder readings |
| LED (Wi-Fi) | 2 | Status indicator for Wi-Fi connection |

### Motor Parameters
- Steps per revolution: 200  
- Microstepping: 1/32  
- Maximum RPM: 400  
- PID gains (tunable):
Kp = 0.04
Ki = 0.002
Kd = 0.0002


---

## Build and Flash Instructions

1. **Set up ESP-IDF (v5.x)** following the official [ESP-IDF documentation](https://docs.espressif.com/).
2. Clone this repository.
3. Build the firmware: idf.py build
4. Flash the ESP32: idf.py flash
5. Monitor before using: idf.py monitor
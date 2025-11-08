# OmniBotCC (Python)

Example project to control an OmniBotCC robot over UDP (ESP32) and demonstrate mapping body velocities to wheel RPMs.

## Repository contents
- `Inicio Rapido.py`  
    Main script with two demos:
    - `simple_demo()` — basic commands (connect, set velocities, start/stop, disconnect) and a sequential RPM motion routine.
    - `vb_demo()` — maps body velocities Vb = [omega, vx, vy] to the 4-wheel RPMs using matrix H0 and controls the robot via UDP.
- `udp_robot.py`  
    UDP module with functions:
    - `robot_connect()` — creates a UDP socket and configures the ESP32 IP/port.
    - `robot_disconnect()` — closes the socket.
    - `robot_start()` / `robot_stop()` — sends `start`/`stop` commands.
    - `robot_set_velocities(rpm_vector)` — sends direction and RPM per motor (`m{i}_dir`, `m{i}_rpm`).
- `robotat.py`  
    Simple TCP connection example (placeholder) with:
    - `robotat_connect()` — attempts a TCP connection to a host/port.
    - `robotat_disconnect(tcp_obj)` — closes the TCP connection.

## Requirements
- Python 3.8+
- numpy
- Standard modules: socket, time, math

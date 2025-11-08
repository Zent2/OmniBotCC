"""
Inicio Rápido del OmniBotCC (Equivalente al archivo de MATLAB "Inicio_Rapido.m")
Autor: Christian Campos
Versión: 1.0
Fecha: 2025-10-13

Requisitos:
 - Python 3.8+
 - numpy
 - udp_robot.py (en el mismo folder) con las funciones:
    robot_connect(), robot_disconnect(), robot_start(), robot_stop(), robot_set_velocities()
"""

import time
import numpy as np
from math import pi
from udp_robot import (
    robot_connect,
    robot_disconnect,
    robot_start,
    robot_stop,
    robot_set_velocities,
)

def simple_demo():
    """Sección: comandos básicos y demostración de movimiento sencillo sin retroalimentación."""
    # Conectar
    robot_connect()

    # Enviar velocidades (aún no mover)
    rpm_vector = [-10 * x for x in [0, 0, 0, 1]]
    robot_set_velocities(rpm_vector)
    input("Velocidades seteadas a {}: Presiona Enter para iniciar movimiento...".format(rpm_vector))
    # Comenzar movimiento con las velocidades guardadas
    robot_start()
    input("Robot en movimiento. Presiona Enter para modificar velocidades...")
    # Modificar velocidades (mientras está en movimiento)
    rpm_vector = [-10 * x for x in [-1, 1, 1, -1]]
    robot_set_velocities(rpm_vector)
    input("Velocidades modificadas a {}: Presiona Enter para detener...".format(rpm_vector))
    # Detener robot (motores aún encendidos)
    robot_set_velocities([0, 0, 0, 0])
    input("Robot detenido. Presiona Enter para apagar motores...")
    # Apagar motores
    robot_stop()
    input("Motores apagados. Presiona Enter para desconectar...")
    # Desconectar
    robot_disconnect()

    # --- Demostración de movimientos secuenciales ---
    robot_connect()

    vel = 40  # rpm base
    input("Presiona Enter para iniciar rutina simple de movimientos...")
    movimientos = [
        ("Adelante",  np.array([1, 1, 1, 1]) * vel),
        ("Atras",     np.array([-1, -1, -1, -1]) * vel),
        ("Derecha",   np.array([1, -1, -1, 1]) * vel),
        ("Izquierda", np.array([-1, 1, 1, -1]) * vel),
        ("Giro CW",   np.array([1, -1, 1, -1]) * vel),
        ("Giro CCW",  np.array([-1, 1, -1, 1]) * vel),
    ]

    time.sleep(1)

    # Velocidades 0 antes de iniciar
    robot_set_velocities([0, 0, 0, 0])

    # Iniciar con velocidades 0
    robot_start()

    for nombre, rpm_vec in movimientos:
        print(f"Movimiento: {nombre}")
        robot_set_velocities(rpm_vec.astype(int).tolist())
        time.sleep(2)     # mantener 2 segundos
        robot_set_velocities([0, 0, 0, 0])
        time.sleep(1)            # pequeña pausa entre movimientos

    robot_stop()
    print("Rutina simple completa.")
    robot_disconnect()

def vb_demo():
    """Sección: demostración con velocidades X, Y y omega (mapear Vb a RPMs)."""
    # Parámetros físicos
    w = ((46.8 + 82.3) / 2) / 1000.0   # m
    l = ((74.65 + 32.35) / 2) / 1000.0 # m
    r = 48 / 1000.0                    # radio rueda [m]

    rpm_max = 80  # límite de RPM para la demo
    # Matriz H0 
    H0 = np.array([
        [-l - w, 1, -1],
        [ l + w, 1,  1],
        [ l + w, 1, -1],
        [-l - w, 1,  1]
    ], dtype=float)

    # Pseudo-inversa
    H0_pinv = np.linalg.pinv(H0)

    # Magnitudes deseadas
    v_linear = 0.12    # m/s
    omega_turn = 0.8   # rad/s
    rpm_max = 400      # actualizar límite

    # Movimientos definidos como Vb = [omega; v_bx; v_by]
    movimientos = [
        ("Adelante",  np.array([0.0,  v_linear,  0.0])),
        ("Atras",     np.array([0.0, -v_linear,  0.0])),
        ("Derecha",   np.array([0.0,  0.0,  v_linear])),
        ("Izquierda", np.array([0.0,  0.0, -v_linear])),
        ("Giro CW",   np.array([-omega_turn, 0.0, 0.0])),
        ("Giro CCW",  np.array([ omega_turn, 0.0, 0.0])),
    ]

    robot_connect()
    # Inicio seguro
    robot_set_velocities([0, 0, 0, 0])
    robot_start()
    time.sleep(0.2)

    for nombre, Vb in movimientos:
        robot_start()
        # Calcular velocidades angulares de rueda en rad/s: u = (1/r) * H0 * Vb
        u_rad_s = (1.0 / r) * (H0 @ Vb)   # vector 4x1

        # Convertir a RPM
        u_rpm = u_rad_s * 60.0 / (2.0 * pi)
        # Si excede el máximo, escalar
        max_abs = np.max(np.abs(u_rpm))
        if max_abs > rpm_max:
            scale = rpm_max / max_abs
            u_rpm = u_rpm * scale

        # Mostrar y enviar
        u_rpm_rounded = np.round(u_rpm).astype(int)
        print(f"Movimiento: {nombre} -> RPMs: [{u_rpm_rounded[0]:.1f} {u_rpm_rounded[1]:.1f} {u_rpm_rounded[2]:.1f} {u_rpm_rounded[3]:.1f}]")
        robot_set_velocities(u_rpm_rounded.tolist())

        # Mantener durante 2s
        time.sleep(2)

        # Parar entre movimientos
        robot_set_velocities([0, 0, 0, 0])
        time.sleep(0.6)

    robot_stop()
    robot_disconnect()
    print("Rutina Vb completa.")

if __name__ == "__main__":
    # Ejecutar el demo completo
    # Comentar o descomentar según se desee ejecutar
    print("== Inicio rápido OmniBotCC (Python) ==")
    simple_demo()
    print("\n== Inicio demo Vb (mapear velocidades corporales a RPM) ==")
    vb_demo()
    print("== Fin de pruebas ==")

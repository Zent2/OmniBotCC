# udp_robot.py

import socket
import time

# Variables globales del módulo
u = None
esp32_ip = None
esp32_port = None

def robot_connect():
    """
    Establece una conexión UDP con el robot.
    """
    global u, esp32_ip, esp32_port

    if u is None:
        esp32_ip = "192.168.50.222"
        esp32_port = 12345

        try:
            u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print('Conexión UDP establecida correctamente.')
        except Exception as e:
            print('ERROR: No se pudo conectar al robot. Verifica que esté encendido.')
            print(e)
    else:
        print('Ya existe una conexión UDP activa.')

    return u, esp32_ip, esp32_port


def robot_disconnect():
    """
    Cierra la conexión UDP con el robot.
    """
    global u
    if u is not None:
        u.close()
        u = None
        print('Conexión UDP cerrada.')
    else:
        print('No hay una conexión UDP activa para cerrar.')


def robot_stop():
    """
    Envía el comando 'stop' al robot.
    """
    global u
    if u is not None:
        try:
            u.sendto(b'stop', (esp32_ip, esp32_port))
            print('Comando de parada enviado al robot.')
        except Exception as e:
            raise RuntimeError('No se pudo enviar el comando de parada al robot.') from e
    else:
        raise RuntimeError('No hay una conexión UDP activa para enviar el comando de parada.')


def robot_start():
    """
    Envía el comando 'start' al robot.
    """
    global u
    if u is not None:
        try:
            u.sendto(b'start', (esp32_ip, esp32_port))
            print('Comando de inicio enviado al robot.')
        except Exception as e:
            raise RuntimeError('No se pudo enviar el comando de inicio al robot.') from e
    else:
        raise RuntimeError('No hay una conexión UDP activa para enviar el comando de inicio.')


def robot_set_velocities(rpm_vector):
    """
    Configura las velocidades de los motores.

    Parámetro:
        rpm_vector (list of float): Vector de 4 elementos con las RPM deseadas.
    """
    global u, esp32_ip, esp32_port

    if u is None:
        raise RuntimeError('No hay conexión activa. Llama a robot_connect() primero.')

    if len(rpm_vector) != 4:
        raise ValueError('Se requiere un vector de 4 elementos para los 4 motores.')

    for i in range(4):
        rpm = abs(rpm_vector[i])
        direction = 'CW' if rpm_vector[i] >= 0 else 'CCW'

        # Limitar RPM al máximo permitido
        rpm = min(rpm, 400)

        # Comando de dirección
        comando_dir = f"m{i}_dir={direction}"
        u.sendto(comando_dir.encode(), (esp32_ip, esp32_port))
        time.sleep(0.05)

        # Comando de RPM
        comando_rpm = f"m{i}_rpm={int(rpm)}"
        u.sendto(comando_rpm.encode(), (esp32_ip, esp32_port))
        time.sleep(0.05)

    print('Velocidades configuradas para los 4 motores vía WiFi.')


def robot_set_ustep(ustepping):
    """
    Configura el microstepping (resolución de paso).

    Parámetro:
        ustepping (int): Valor deseado de microstepping (por ejemplo, 1, 2, 4, 8...).
    """
    global u, esp32_ip, esp32_port

    if u is None:
        raise RuntimeError('No hay conexión activa. Llama a robot_connect() primero.')

    comando_ustepping = f"step={ustepping}"
    u.sendto(comando_ustepping.encode(), (esp32_ip, esp32_port))
    print(f'Ustepping configurado a {ustepping}.')

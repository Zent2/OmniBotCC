# udp_connection.py

import socket

# Variables globales del módulo
u = None
esp32_ip = None
esp32_port = None

def robot_connect():
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
    
    return u, esp32_ip, esp32_port  # Para usar el socket desde otro archivo

import socket

def robotat_connect():
    ip = '192.168.50.200'
    port = 1883
    try:
        tcp_obj = socket.create_connection((ip, port))
        return tcp_obj
    except Exception as e:
        print("ERROR: Could not connect to Robotat server.")
        return None

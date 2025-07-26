# robotat.py

import socket

def robotat_connect():
    ip = '192.168.50.200'
    port = 1883
    try:
        tcp_obj = socket.create_connection((ip, port), timeout=3)
        return tcp_obj
    except Exception as e:
        print("ERROR: Could not connect to Robotat server.")
        print(e)
        return None


def robotat_disconnect(tcp_obj):
    if tcp_obj:
        tcp_obj.close()
        print("Disconnected from the robot.")
    else:
        print("No active connection to disconnect from.")
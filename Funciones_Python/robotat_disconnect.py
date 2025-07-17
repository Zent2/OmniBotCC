def robotat_disconnect(tcp_obj):
    if tcp_obj:
        tcp_obj.close()
        print("Disconnected from the robot.")
    else:
        print("No active connection to disconnect from.")
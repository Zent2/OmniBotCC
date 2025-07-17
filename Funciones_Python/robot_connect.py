def robot_connect(tcp_obj, robot_id):
    cmd = f'connect,{robot_id}\n'
    tcp_obj.send(cmd.encode())

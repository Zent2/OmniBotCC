import udp_robot

udp_robot.robot_connect()
udp_robot.robot_start()
udp_robot.robot_set_velocities([150, -150, 100, -100])
udp_robot.robot_stop()
udp_robot.robot_disconnect()

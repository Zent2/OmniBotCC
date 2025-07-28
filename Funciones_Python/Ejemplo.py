import udp_robot
import robotat
import time
vel=400
# tcp_connection = robotat.robotat_connect()

udp_robot.robot_connect()

udp_robot.robot_set_ustep(32)
udp_robot.robot_set_velocities([0, vel, 0, 0])
udp_robot.robot_start()


time.sleep(60)
#udp_robot.robot_set_ustep(32)
#udp_robot.robot_set_velocities([-1, 1, -1, 1])
#time.sleep(2)
udp_robot.robot_stop()
udp_robot.robot_disconnect()
# robotat.robotat_disconnect(tcp_connection)

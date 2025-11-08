
# <span style="color:rgb(213,80,0)">OmniBotCC – MATLAB Project Documentation</span>

This Live Script provides an overview of the **OmniBotCC** MATLAB environment,


used for trajectory generation, robot control, and pose tracking with


the **Robotat** motion capture system.


The project communicates with an omnidirectional mobile robot equipped


with a ESP32 microcontroller and integrates MATLAB for simulation, closed loop control,


trajectory planning and performance evaluation.


**Author**: Christian Campos


**Date:** October 2025  

# Project Overview

This MATLAB workspace contains all the scripts and functions needed for:

-   Establishing communication with the omnidirectional robot via Wi-Fi. 
-   Tracking and recording the robot’s pose using the Robotat system. 
-   Generating and executing pre-defined and planed trajectories. 
-   Visualizing, calibrating, and analyzing system performance. 


 The workflow typically follows this sequence:

1.   Connection.
2. Trajectory Generation
3. Trajectory Execution or Marker Tracking in closed loop
4. Data Visualization and Error Analysis

#  Directory Structure

 | `/Simulaciones` | Folder | Simulation models and test scenarios |


 | `/slprj` | Folder | Auto-generated Simulink project files |


 | `Calibracion_Robot.m` | Script | Calibrates the robot’s position and angle (yaw) |


 | `genmap.m` | Script | Generates a Robotat map for trajectory planning |


 | `Inicio_Rapido.m` | Script | Quick Start |


  | `Offsets.mat` | Data | Pre-saved position offsets from Robotat calibration |


  | `q2eul.m`, `q2rot.m` | Functions | Convert quaternions to Euler angles or rotation matrices |


 | `robotat_connect.m` | Function | Connects to the Robotat motion capture server |


 | `robotat_disconnect.m` | Function | Ends connection with Robotat |


 | `robotat_get_pose.m` | Function | Retrieves the current robot pose from Robotat |


 | `robotat_trvisualize.m` | Function | Visualizes rigid bodies within the Robotat reference frame |


 | `robot_connect.m` | Function | Establishes communication with the robot controller |


 | `robot_disconnect.m` | Function | Closes communication with the robot |


 | `robot_emergency_stop.m` | Function | Immediately halts robot movement |


 | `robot_send_WiFi.m` | Function | Sends data packets to the robot over Wi-Fi |


 | `robot_set_ustep.m` | Function | Sets the microstepping configuration for motors | (NOT USED)


 | `robot_set_velocities.m` | Function | Sends velocity commands to each wheel |


 | `robot_start.m` / `robot_stop.m` | Functions | Start/stop robot motion commands |


 | `Trayectoria_Omnidireccional.m` | Script | Omnidirectional trajectory generation and execution module |


 | `Trayectorias_Mapa.m` | Script | Builds and visualizes a trajetory over a map with obstacles|


 | `Seguimiento_marcador.m` | Script | Follow a markers' position |

#  Simulation Environment

 The `/Simulaciones` folder includes models and scripts for testing


 the robot control algorithms in a simulated environment before


 applying them to the real hardware.

#  License

 This project is property of Universidad del Valle de Guatemala.


```matlab
mdfile = export("README.mlx",Format="markdown")
```

```TextOutput
mdfile = 'C:\Users\chris\OneDrive - UVG\UVG-ChrisLaptop\Proyecto de Graduacion-Tesis\Carpeta Final\Funciones_Matlab\README.md'
```

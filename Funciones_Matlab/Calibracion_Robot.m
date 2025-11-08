% -----------------------------------------------------------------------------
% Calibración de los offsets para el OmniBotCC
% -----------------------------------------------------------------------------
% Propósito
% Definir los "offsets" en x,y,z y yaw del OmniBotCC.
%
% Uso rápido
%   1) Ejecución del script sección por sección. La primera sección es de
%   visualización del robot dentro del Robotat. Debería poder realizarse
%   la calibración al colocar el Robot en el centro del Robotat.
%   2) Se coloca el Robot en el centro del Robotat, alineando el eje X del
%   Robot con el Eje X del Robotat.
%   3) Se ejecuta la segunda sección del código, y se guardan los offsets
%   en un archivo para poder llamarlos en otro código.
%   4) En la tercera parte se hace la prueba para comprobar que la
%   configuración se realizó correctamente. Mover el Robot a una medida y
%   rotación conocida y comprobar la configuración.
%
% Variables importantes (entrada / salida)
%   Entradas configurables:
%     - ID: ID del cuerpo rígido dentro de la configuración de "motion"
%     (Robotat).
%
% Comportamiento y supuestos
%   - El Robot se calibrará en el centro del Robotat.
%
% Tips rápidos de uso / debugging
%   - Verifique que el Robotat tenga espacio en el centro y que la
%   orientación y posición sea la adecuada.
%
% Autores / versión
%   - Autor: Christian Campos 
%   - Versión: 1.0
%   - Fecha: 2025-10-13
% -----------------------------------------------------------------------------

disp("----------Calibración del Robot Omnidireccional dentro del Robotat----------");
%% Midiendo el robot dentro del robotat
disp("Visualizando el Robot dentro del Robotat (sin offsets)");
%Conectar al robotat
robotat=robotat_connect(); 
id=29;
%Visualizar el robot 
robotat_trvisualize(robotat,id);

%% Obtener posición de los markers
disp("Calibrando el Robot");
pos_markers = robotat_get_pose(robotat, id, 'eulzyx');

xpos_markers = pos_markers(1) * 1000; % en mm
ypos_markers = pos_markers(2) * 1000; % en mm
yaw_markers = wrapTo180(atan2d(sind(pos_markers(4)), cosd(pos_markers(4)))); %WrapTo180
z_offset=(82/2)+31; %En mm
offsets_mm=[xpos_markers, ypos_markers, z_offset ,yaw_markers,0,0];
offsets_m=[offsets_mm(1)/1000, offsets_mm(2)/1000, offsets_mm(3)/1000 ,yaw_markers,0,0];
save 'Offsets' offsets_mm offsets_m;
%% Para la posición real
disp("Comprobando calibración");
load Offsets;
offset=offsets_m;
%robot_pos=[offsets_mm(1:3)/1000 offsets_mm(4)];
robot_pos = robotat_get_pose(robotat, id, 'eulzyx');
x_real = (robot_pos(1) - offset(1));   % m
y_real = (robot_pos(2) - offset(2));   % m
yaw_real=atan2d(sind(robot_pos(4)), cosd(robot_pos(4)))-offset(4);

robot_pos_real=[x_real y_real offset(3) yaw_real]
robotat_disconnect(robotat);

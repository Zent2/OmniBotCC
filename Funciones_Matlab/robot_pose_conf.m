%% OmniRobot Robotat 
%% Midiendo el robot dentro del robotat
%Conectar al robotat
robotat=robotat_connect(); 
id=29;
%Visualizar el robot (para centrarlo)
robotat_trvisualize(robotat,id);

%% Obtener posición de los markers
pos_markers = robotat_get_pose(robotat, id, 'eulzyx');

offset=0;
xpos_markers = pos_markers(1) * 1000; % en mm
ypos_markers = pos_markers(2) * 1000; % en mm
theta_markers = atan2d(sind(pos_markers(4) - offset), cosd(pos_markers(4) - offset));

%save 'Valores_markers' theta_markers ypos_markers xpos_markers
%% Para el ángulo

% Offsets de los markers (Pololus) (medidos para que el yaw sea cero cuando cada
% robot esté alineado al eje +x del Robotat)
%marker_offsets = [-93.7618, -89.7361, -89.8177, -81.9006, -88.8106, ...
%    -89.9653, -94.0866, -89.9872, -94.2587, 33.6573];
load Valores_markers.mat
offset=[xpos_markers ypos_markers theta_markers];
offset_z=0.048/2; %En cm
%robot_pos = robotat_get_pose(robotat, robot_no, 'eulzyx');
robot_pos=[offset(1),offset(2),offset_z, offset(3)];
xpos = (robot_pos(1)-offset(1))* 1000; % en mm
ypos = (robot_pos(2)-offset(2)) * 1000; % en mm
theta = atan2d(sind(robot_pos(4) - offset(3)), cosd(robot_pos(4) - offset(3)));
%
%% Para la posición

% Ejemplo MaxArm%% Transformada Homogenea
% pose_marker = robotat_get_pose(robotat, id);  % Obtiene [x, y, z, qx, qy, qz, qw]
% 
% p = pose_marker(1:3);           % Posición
% q = pose_marker(4:7);           % Cuaternión (qx, qy, qz, qw)
% R = q2rot(q);               % Convierte cuaternión a matriz de rotación
% 
% I_T_M = [R, p(:);                   % Matriz de rotación + vector columna
%      0 0 0 1];                  % Parte homogénea

% %% Transformadas
% zoff=2.75;
% I_T_b0=diag([1, 1, 1, 1]);
% I_T_b0(3,4)=zoff;
% %% Pose de la base del robot con respecto de la base con markers
% 
% M_T_b=inv(I_T_M)*I_T_b0;

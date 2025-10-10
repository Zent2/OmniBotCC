%% OmniRobot Robotat 
%% Midiendo el robot dentro del robotat
%Conectar al robotat
robotat=robotat_connect(); 
id=29;
%Visualizar el robot 
robotat_trvisualize(robotat,id);

%% Obtener posición de los markers
pos_markers = robotat_get_pose(robotat, id, 'eulzyx');

xpos_markers = pos_markers(1) * 1000; % en mm
ypos_markers = pos_markers(2) * 1000; % en mm
yaw_markers = atan2d(sind(pos_markers(4)), cosd(pos_markers(4)));
z_offset=(82/2)+31; %En mm
offsets_mm=[xpos_markers, ypos_markers, z_offset ,yaw_markers,0,0];
offsets_m=[offsets_mm(1)/1000, offsets_mm(2)/1000, offsets_mm(3)/1000 ,yaw_markers,0,0];
save 'Offsets' offsets_mm offsets_m;
%% Para la posición real

% Offsets de los markers (Pololus) (medidos para que el yaw sea cero cuando cada
% robot esté alineado al eje +x del Robotat)
%marker_offsets = [-93.7618, -89.7361, -89.8177, -81.9006, -88.8106, ...
%    -89.9653, -94.0866, -89.9872, -94.2587, 33.6573];
load Offsets;
%offset=[xpos_markers ypos_markers theta_markers];
%offset_z=0.048/2; %En mm
robot_pos = robotat_get_pose(robotat, id, 'eulzyx');
offset=offsets_m;
%robot_pos=[offset(1),offset(2),offset(3), offset(4)];
xpos = (robot_pos(1)-offset(1))* 1000; % en mm
ypos = (robot_pos(2)-offset(2)) * 1000; % en mm
yaw = atan2d(sind(robot_pos(4) - offset(4)), cosd(robot_pos(4) - offset(4)));
robot_pos_real=[xpos ypos offsets_mm(3) yaw]
%%
robotat_disconnect(robotat);


robot_connect(); 
%%
rpm_vector = -1*[0 1 0 0];
robot_set_velocities(rpm_vector);
clear toc;
% Iniciar temporizador
tic;
robot_start();
while toc < 60
    count=0;
end

robot_stop();
%%
robot_disconnect();

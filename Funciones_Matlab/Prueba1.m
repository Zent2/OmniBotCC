robot_connect(); 
%%
rpm_vector = -25*[1 -1 1 1];
robot_set_velocities(rpm_vector);
%clear toc;
% Iniciar temporizador
%tic;
%%
robot_start();
%% Kp, Ki, Kd
robot_stop();
%%
robot_send_WiFi("m1_dir=CCW");
robot_send_WiFi("m1_rpm=10");
robot_send_WiFi("m1_start");
%%
robot_send_WiFi("m2_dir=CCW");
robot_send_WiFi("m2_rpm=100");
robot_send_WiFi("m2_start");
%%
robot_send_WiFi("m2_stop");
robot_send_WiFi("m2_dir=CW");
%%
robot_send_WiFi("Kp=0.040");
robot_send_WiFi("Ki=0.004");
robot_send_WiFi("Kd=0.0002");


%%
rpm_vector = -(300)*[1 1 1 1];
robot_set_velocities(rpm_vector);
clear toc;
% Iniciar temporizador
tic;
robot_start();
while toc < 8
    count=0;
end

robot_stop();
%%
robot_disconnect();

%% Prueba de envio

robot_set_ustep(32);

robot_stop();
robot_connect(); 
%%
% Velocidad base en rpm (ajusta según tu robot)
vel = 40;

% Definición de los vectores de velocidad de cada rueda
% Orden: [m1 m2 m3 m4] (ajusta si tu robot tiene otra convención)

movimientos = {
    'Adelante',        vel*[1 1 1 1];
    'Atrás',           -vel*[1 1 1 1];
    'Izquierda',       vel*[-1 1 -1 1];
    'Derecha',          -vel*[-1 1 -1 1];
    'Diagonal Ad-izq',  vel/2*[0 1 0 1];
    'Diagonal At-izq',  -vel/2*[1 0 1 0];
    'Diagonal Ad-der',  vel/2*[1 0 1 0];
    'Diagonal At-der',  -vel/2*[0 1 0 1];
    'Rotar CW',        vel*[-1 1 1 -1];
    'Rotar CCW',        vel*[1 -1 -1 1];
    'Prueba1' [17.4513 62.1261 22.3374 57.2401]; 
};
pause(2);
% Recorrer todos los movimientos
for k = 1:size(movimientos,1)
    nombre = movimientos{k,1};
    rpm_vector = movimientos{k,2};

    disp(['Movimiento: ', nombre]);

    % Asignar velocidades
    robot_set_velocities(rpm_vector);

    % Iniciar robot
    robot_start();
    tic;
    while toc < 2
        % loop de espera de 2 segundos
    end
    robot_stop();
    pause(0.1);
    robot_stop();
    pause(1); % pequeña pausa entre movimientos
end

disp('Rutina completa.');
%%
robot_disconnect();


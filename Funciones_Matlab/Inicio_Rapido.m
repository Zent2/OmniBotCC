% =========================================================================
% Inicio Rápido del OmniBotCC
% =========================================================================
% Propósito
%   % Mostrar al usuario los comandos básicos del OmniRobotCC. Incluye: 
%   % - Demostración de conexión y desconexión al Robot.
%   % - Demostración de inicio, parada y seteo de velocidades.
%
% Uso rápido
%   1) Ejecución del script sección por sección (antes de la última sección) para visualizar 
%   el correcto funcionamiento de los comandos. 
%   2) Ejecución de la última sección para visualizar un movimiento simple
%   sin retroalimentación.
%
% Variables importantes (entrada / salida)
%   Entradas configurables:
%     - Vel (rpms)
%
% Comportamiento y supuestos
%   - El Robot debería tener suficiente espacio para moverse durante 2
%   segundos hacia adelante e intentar regresar a su punto de inicio. Al no
%   tener retroalimentación no se asegura que esto sea cierto.
%   - El código no es de control, solo de demostración.
%
% Tips rápidos de uso / debugging
%   - Verifique que el Robot tenga espacio a sus alrededores.
%   - Verifique que el Robot este conectado al Wi-Fi del Robotat, esto lo
%   indica la LED Azul del ESP32.
%   - Si el ESP32 no conecta, reinicie el ESP32 y compruebe el estado del
%   Wi-Fi del Robotat.
%
% Autores / versión
%   - Autor: Christian Campos 
%   - Versión: 1.0
%   - Fecha: 2025-10-13
% -----------------------------------------------------------------------------

%% Comando para conexión con el Robot
robot_connect(); 

%% Comando para enviar velocidades al Robot (aún no está en movimiento)
rpm_vector = -10*[0 0 0 1];
robot_set_velocities(rpm_vector);

%% Comando para comenzar el movimiento del Robot con las velocidades guardadas
robot_start(); 

%% Comando para enviar velocidades al Robot (modifica las velocidades actuales)
rpm_vector = -10*[-1 1 1 -1];
robot_set_velocities(rpm_vector);

%% Comando para detener el Robot (los motores siguen encendidos)
rpm_vector = [0 0 0 0];
robot_set_velocities(rpm_vector);

%% Comando para detener el Robot y apagar los motores.
robot_stop();

%% Comando para desconexión con el Robot
robot_disconnect();

%% Demostración movimiento sencillo sin retroalimentación.

robot_connect(); 

% Velocidad base en rpm (se puede ajustar hasta a 400rpms)
vel = 40;

% Definición de los vectores de velocidad de cada rueda
% Orden: [m1 m2 m3 m4]
movimientos = { 
    'Adelante',   vel * [  1   1   1   1 ];   % +Y (avanza hacia adelante)
    'Atrás',      vel * [ -1  -1  -1  -1 ];   % -Y (retrocede)
    'Derecha',    vel * [  1  -1  -1   1 ];   % +X (desplaza a la derecha)
    'Izquierda',  vel * [ -1   1   1  -1 ];   % -X (desplaza a la izquierda)
    'Giro CW',    vel * [  1  -1   1  -1 ];   % Rotación horario
    'Giro CCW',   vel * [ -1   1  -1   1 ];   % Rotación antihorario
};


pause(1);

% Velocidades 0 antes de iniciar
robot_set_velocities([0 0 0 0]);

% Iniciar robot con velocidades 0
robot_start();

% Vamos a realizar todos los movimientos
for k = 1:size(movimientos,1)
    nombre = movimientos{k,1};
    rpm_vector = movimientos{k,2};

    disp(['Movimiento: ', nombre]);

    % Asignar velocidades
    robot_set_velocities(rpm_vector);
    tic;
    while toc < 2
        % loop de espera de 2 segundos
    end
    % Detener Robot (sin apagar motores)
    robot_set_velocities([0 0 0 0]);
    pause(1); % pequeña pausa entre movimientos
end

% Detener Robot
robot_stop();

disp('Rutina completa.');

% Desconectar Robot
robot_disconnect();

%% Demostración con velocidades X y Y 
% Parámetros fí­sicos del robot 
w = ((46.8+82.3)/2)/1000;   % m
l = ((74.65+32.35)/2)/1000; % m
r = 48/1000;                % radio rueda [m]
% Lí­mites motores
rpm_max = 80;    % rpm

H0 = [ -l - w, 1, -1;
        l + w, 1,  1;
        l + w, 1, -1;
       -l - w, 1,  1 ];
H0_pinv = pinv(H0);

% Magnitudes deseadas 
v_linear = 0.12;    % m/s -> velocidad lineal de referencia para traslación
omega_turn = 0.8;   % rad/s -> velocidad angular de referencia para giro
rpm_max=400;
% Nota sobre convenciones:
% Vb = [ omega; v_bx; v_by ] donde
%  - v_bx = avance (forward) en m/s
%  - v_by = lateral (derecha positiva) en m/s
%  - omega: rotación positiva = CCW 

% Mapear movimientos a Vb
movimientos = {
    'Adelante',   [0;  v_linear;  0];    % avanzar +X del cuerpo (forward)
    'Atrás',      [0; -v_linear;  0];
    'Derecha',    [0;  0;  v_linear];    % desplazamiento lateral +Y del cuerpo
    'Izquierda',  [0;  0; -v_linear];
    'Giro CW',    [-omega_turn; 0; 0];    % giro horario --> omega negativo 
    'Giro CCW',   [ omega_turn; 0; 0];
};
robot_connect(); 
% Inicio seguro
robot_set_velocities([0 0 0 0]);
robot_start();
pause(0.2);

for k = 1:size(movimientos,1)
    robot_start();
    nombre = movimientos{k,1};
    Vb = movimientos{k,2}; % [omega; v_bx; v_by]
    
    % Calcular velocidades angulares de ruedas (rad/s)
    % u_rad_s = (1/r) * H0 * Vb
    u_rad_s = (1/r) * (H0 * Vb);
    
    % Convertir a RPM
    u_rpm = u_rad_s * 60 / (2*pi);
    
    
    if any(abs(u_rpm) > rpm_max)
        scale = rpm_max / max(abs(u_rpm));
        u_rpm = u_rpm * scale;
    end
    
    
    fprintf('Movimiento: %s -> RPMs: [%.1f %.1f %.1f %.1f]\n', ...
        nombre, u_rpm(1), u_rpm(2), u_rpm(3), u_rpm(4));
    robot_set_velocities(round(u_rpm(:))');
    
    % Mantener durante 2 s
    tic;
    while toc < 2
        % espera activa
        pause(0.01);
    end
    
    % Parar entre movimientos
    robot_set_velocities([0 0 0 0]);
    pause(0.6);
end

robot_stop();
robot_disconnect();
disp('Rutina completa.');



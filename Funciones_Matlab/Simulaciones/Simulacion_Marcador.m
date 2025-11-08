% -----------------------------------------------------------------------------
% Simulación - Seguimiento de punto aleatorio con plataforma omnidireccional 
% -----------------------------------------------------------------------------
% Propósito
%   Simular y visualizar el seguimiento de un punto aleatorio de la plataforma 
%   omnidireccional de 4 ruedas OmniRobotCC. Incluye:
%     - Punto aleatorio: Punto aleatorio a seguir por la plataforma móvil.
%     - Controlador PI en velocidades del cuerpo + modelo simple de motores.
%     - Animación en tiempo real y gráficos de resumen.
%
% Uso rápido
%   1) Ajuste parámetros en el bloque "Inicio" (start_world) y "Parámetros de la 
%   simulación" (Kp, Ki).
%   2) Ejecute el script. 
%   3) Observe animación y gráficos finales; revise variables de salida.
%
% Variables importantes (salida / entradas)
%   Entradas configurables (en el script):
%     - start_world : [x y] posición inicial simulada (m)
%     - numero_marcadores: n cantidad de "marcadores" aleatorios
%     - dt, Tsim, Kp, Ki, rpm_max, lookahead_dist
%   Salidas / logs:
%     - Xlog    : poses simuladas durante la ejecución
%     - rpm_log : rpm reales simulados por rueda
%     - rpm_cmd_log : rpm comandos enviados al modelo
%     - Errlog  : errores registrados (posición, yaw)
%
% Comportamiento y supuestos
%   - No controla hardware: es una simulación. Para integrar hardware, sustituir
%     modelos por llamadas robot_* y verificar conversiones de unidades y ángulos.
%   - Ángulos: todas las operaciones trigonométricas usan radianes.
%
% Tips rápidos de uso / debugging
%   - Si el robot gira mal: compruebe rotate_policy y kp_phi/ki_phi.
%   - Para pruebas sin rotación, usar rotate_policy = 'never'.
%   - Para depurar control, reduzca Tsim y aumente dt (más rápido para iterar).
%
% Autores / versión
%   - Autor: Christian Campos
%   - Versión: 1.0 
%   - Fecha: 2025-10-13
% -----------------------------------------------------------------------------

close all; clear; clc;
disp("----------Simulación Seguimiento de un marcador en tiempo real con Control " + ...
    "FeedForward+PI Robot Omnidireccional----------");
%% -------------------- Parámetros del mundo / mapa --------------------
map_w = 4.0;   % ancho en metros (X)
map_h = 5.0;   % alto en metros (Y)
% Origen del mundo: colocamos (0,0) en el centro del mapa para la simulaci?n
x_min = -map_w/2; x_max = map_w/2;
y_min = -map_h/2; y_max = map_h/2;

%% ----- Definir obstáculos (centros en m) y visualizar mapa -----
% Generar mapa 
obstacles=[];
map=genmap(obstacles);
%figure(2);
%imshow(map);
%title('Mapa actual Robotat', ...
   % 'FontSize', 14, 'Interpreter', 'latex');
%xlabel('$x$ (en cm)', 'FontSize', 14, 'Interpreter', 'latex');
%ylabel('$y$ (en cm)', 'FontSize', 14, 'Interpreter', 'latex');

%legend();
%axis on;

%% -------------------- Inicio (en metros) --------------------
start_world = [0.0, 0.0];   % [x,y] 200, 250 


%% -------------------- Objetivo aleatorio  --------------------
% Parámetros para selector de objetivo aleatorio
min_marker_dist_m = 0.5;   % distancia mínima desde el robot al marker (m).
max_marker_dist_m = 1.0;   % distancia mínima desde el robot al marker (m).
max_tries = 200;            % reintentos para encontrar punto válido

% Función inline para generar punto aleatorio dentro de los límites del Robotat
rand_point = @() [ x_min + (x_max-x_min)*rand(),  y_min + (y_max-y_min)*rand() ];

% Generar marker inicial (asegurando distancia mínima al inicio)
marker = rand_point();
tries = 0;
while norm(marker - start_world) < min_marker_dist_m && tries < max_tries
    marker = rand_point();
    tries = tries + 1;
end
if tries >= max_tries
    warning('No se encontró punto suficientemente alejado del inicio; usando el primer candidato.');
end
fprintf('Marker inicial en [%.3f, %.3f] m\n', marker(1), marker(2));
% Nota: marker está en metros. En el bucle se usará xg = marker(1); yg = marker(2);

%% -------------------- Parámetros fí­sicos del robot --------------------
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

%% Parámetros de la simulación
% Simulación
dt = 0.02;        % s
Tsim = 30;        % s
time = 0:dt:Tsim;

% Configuración bloqueo rotación (definir ANTES del for)
rotate_policy = 'never'; % opciones: 'only_at_goal', 'never', 'always'
rotate_fraction = 0.7; % si policy 'late'

% Estado inicial
state = [start_world(1); start_world(2); 0]; % [x;y;theta]

% Control (pure pursuit + PID exponencial/PI en velocidad cuerpo)
control = "PI"; 

% Pure pursuit param
lookahead_dist = 190/2; % mm

% PID en velocidad cuerpo (PI) - ganancias iniciales (simulación)
kp_phi = 3.2; kp_x = 3.2; kp_y = 3.2;
ki_phi = 0.8; ki_x = 1.6; ki_y = 1.6;
Kp = diag([kp_x, kp_y, kp_phi]);
Ki = diag([ki_x, ki_y, ki_phi]);

%Matriz de error
Ierr = zeros(3,1);
I_max = [1.0; 0.5; 0.5]; I_min = -I_max;
eP=10000;
eO=deg2rad(180);
% Para visualización del robot
robot_w = 0.19; robot_h = 0.19; % m
base_rect = [-robot_h/2, -robot_w/2;
              robot_h/2, -robot_w/2;
              robot_h/2,  robot_w/2;
             -robot_h/2,  robot_w/2]';

%% Figura del mapa-las trayectorias y la animación de la plataforma móvil
fig_map = figure('Name','Mapa','Color','w');
axis equal; axis([-map_w/2 map_w/2 -map_h/2 map_h/2]); grid on; hold on;
xlabel('X [m]'); ylabel('Y [m]'); title('Animación Robot');

hPatch = patch('XData',[], 'YData',[], 'FaceColor',[.6 .8 .9], 'EdgeColor','k');
hTrail = animatedline('Color','r','LineWidth',1.5);
hMeta = plot(marker(1), marker(2), '*r', 'MarkerSize',10, 'LineWidth',1.5);
% visual heading (flecha que muestra el eje X del robot)
heading_len = 0.19/2; % longitud de la flecha en metros 
hHeading = quiver(0,0,0,0,0, 'LineWidth', 2, 'MaxHeadSize', 0.6, ...
                  'Color', [0 0 0.6], 'AutoScale', 'off'); % inicializa vacío

hTime = text(-map_w/2+0.05, map_h/2-0.15, '', 'FontSize', 10, 'FontWeight','bold');
hVelRuedas = text(map_w/2-0.6, map_h/2-0.3, '', 'FontSize',9, 'FontName','Courier New');

%% Animación de la plataforma

% Variables auxiliar
t = 0;
% Variables de logging
Xlog = zeros(3,length(time));
rpm_log = zeros(4,length(time));
Errlog = zeros(3,length(time));

% Parámetros motor (modelo de primer orden)
rng(0); % reproducible
u_rpm_meas = zeros(4,1);       % estado interno: rpm reales
tau_motor = 0.12;              % [s]
motor_gain = 0.98 + 0.04*randn(4,1);  % ganancia por rueda
motor_bias = 0.5 * randn(4,1);        % bias [rpm]
motor_noise_std = 100.0;         % ruido [rpm] (std)
r_wheels = r .* (1 + 0.005*randn(4,1)); % pequeñas diferencias de radio

seguimiento=0;
numero_marcadores=3;
% loop principal (pure-pursuit + control)
for k = 1:length(time)
    t = time(k);

    % --------- 2) Objetivo dinámico: marker aleatorio dentro del área ----------
    % marker está en metros; xg, yg serán usados en el control
    xg = marker(1); yg = marker(2);

    % Mostrar el marker en la figura:
    set(hMeta, 'XData', xg, 'YData', yg);

    

    % ---------- Si el robot está dentro del lookahead_dist -> elegir nuevo marker ----------
    if (eP/1000 < 0.75*min_marker_dist_m || (seguimiento==0)) && (seguimiento<numero_marcadores)
        fprintf('Robot cercano al marker (eP=%.1f mm). Seleccionando nuevo marker...\n', eP);
        % Generar nuevo marker aleatorio no demasiado cercano al robot
        new_marker = rand_point();
        tries = 0;
        while norm(new_marker - state(1:2)') < min_marker_dist_m &&...
                norm(new_marker - state(1:2)') > max_marker_dist_m && tries < max_tries
            new_marker = rand_point();
            tries = tries + 1;
        end
        if tries >= max_tries
            warning('No se encontró nuevo marker suficientemente alejado; usando candidato.');
        end
        marker = new_marker; % actualizar objetivo
        % Actualizar plot del marker
        set(hMeta, 'XData', marker(1), 'YData', marker(2));
        % (No hacemos break: la simulación continúa con el nuevo objetivo)
        seguimiento=seguimiento+1;
    end

    block_theta = false;
    switch rotate_policy
        case 'only_at_goal'
            if ~((eP < 8*lookahead_dist) && (seguimiento>=numero_marcadores))
                block_theta = true;
            end
        case 'never'
            block_theta = true;
        otherwise
            block_theta = true;
    end
    % pose actual
    x = state(1); y = state(2); th = state(3);

    
    % ------------------ Control PI ------------------
    % errores (en mm para eP)
    e = [xg - x; yg - y];
    eP = norm(e)*1000; % mm

    % orient. deseada
    if block_theta
        thetag = 0; %Alinear al eje X
        
    else
        thetag = atan2(e(2), e(1));
        
    end
    
    eO = wrapToPi(thetag - th);
    err = [ xg - x; yg - y; eO ];

    Ierr_cand = Ierr + err * dt;
    Ierr_cand = max(min(Ierr_cand, I_max), I_min);
    

    v_des = 2.0 * eP/1000;   % m/s
    w_des = 0.2 * eO;        % rad/s
    qd_dot_world = [ v_des*cos(thetag); v_des*sin(thetag); w_des ];

    qdot_world = qd_dot_world + Kp * err + Ki * Ierr_cand;
    % convertir mundo -> cuerpo
    R_th = [cos(th), -sin(th); sin(th), cos(th)];
    v_body = R_th' * qdot_world(1:2);
    omega = qdot_world(3);

    Vb_cmd = [omega; v_body(1); v_body(2)];
    % actualizar integrador solo si no se satura (posterior)


    % ------------------ Ruedas: comando (rad/s -> rpm) ------------------
    u_rad_s_cand = (1/r) * H0 * Vb_cmd;       % rad/s por rueda (ideal)
    u_rpm_cand = u_rad_s_cand * 60 / (2*pi);  % rpm (ideal comando)

    % saturación y anti-windup
    if any(abs(u_rpm_cand) > rpm_max)
        scale = rpm_max / max(abs(u_rpm_cand));
        u_rpm_cmd = u_rpm_cand * scale;
        u_rad_s = u_rpm_cmd * 2*pi / 60;
        Vb_real_from_cmd = r * H0_pinv * u_rad_s;
        % no actualizar integrador
    else
        u_rpm_cmd = u_rpm_cand;
        u_rad_s = u_rpm_cmd * 2*pi / 60;
        Vb_real_from_cmd = Vb_cmd;
        Ierr = Ierr_cand;
    end

    % Guardar comando
    rpm_cmd_log(:,k) = u_rpm_cmd;

    % ------------------ Modelo motor (primer orden, simple y realista) ------------------
    % target que el motor "trata" de alcanzar (ganancia y bias)
    target_rpm = motor_gain .* u_rpm_cmd + motor_bias;

    % Euler discretizado: du/dt = (target - u)/tau + ruido
    noise = motor_noise_std * sqrt(dt) .* randn(4,1);
    u_rpm_meas = u_rpm_meas + (dt / tau_motor) .* (target_rpm - u_rpm_meas) + noise;

    % deadband por fricción estática
    deadband = 0.4;
    u_rpm_meas(abs(u_rpm_meas) < deadband) = 0;
    % convertir a rad/s y calcular Vb_real considerando radios individuales
    u_rad_s_meas = u_rpm_meas * 2*pi / 60;
    Vb_real = H0_pinv * (r_wheels .* u_rad_s_meas);    % 3x1

    % Integración cinemática global
    omega_bz = Vb_real(1);
    v_bx = Vb_real(2);
    v_by = Vb_real(3);

    vx_world = v_bx*cos(th) - v_by*sin(th);
    vy_world = v_bx*sin(th) + v_by*cos(th);

    state(1) = state(1) + vx_world*dt;
    state(2) = state(2) + vy_world*dt;
    state(3) = wrapToPi(state(3) + omega_bz*dt);

    % Visualización en tiempo real
    pos = [cos(state(3)), -sin(state(3)); sin(state(3)), cos(state(3))] * base_rect + state(1:2);
    set(hPatch, 'XData', pos(1,:), 'YData', pos(2,:));
    addpoints(hTrail, state(1), state(2));
    set(hMeta, 'XData', xg, 'YData', yg);
    % actualizar flecha de heading
    hx = state(1);
    hy = state(2);
    ux = heading_len * cos(state(3));   % componente x de la flecha en m
    uy = heading_len * sin(state(3));   % componente y de la flecha en m
    set(hHeading, 'XData', hx, 'YData', hy, 'UData', ux, 'VData', uy);

    v_display = norm([Vb_real(2), Vb_real(3)]);
    w_display = Vb_real(1);
    set(hTime, 'String', sprintf('Tiempo: %.2f s | eP=%.0f mm | eO=%.2f deg', t, eP, rad2deg(eO)));
    set(hVelRuedas, 'String', sprintf(['v=%.3f m/s  w=%.3f rad/s\nRPM cmd: %6.0f %6.0f ' ...
        '%6.0f %6.0f\nRPM real:%6.0f %6.0f %6.0f %6.0f'],...
        v_display, w_display, u_rpm_cmd(1),u_rpm_cmd(2),u_rpm_cmd(3),u_rpm_cmd(4),...
        u_rpm_meas(1),u_rpm_meas(2),u_rpm_meas(3),u_rpm_meas(4)));

    drawnow limitrate;
    pause(dt);

    % Logging
    Xlog(:,k) = state;
    rpm_log(:,k) = u_rpm_meas;
    Errlog(:,k) = [eP; eO; 0];

    % condición de parada si meta final alcanzada
    if eP < lookahead_dist && norm(eO)<deg2rad(15) % mm
        disp('Objetivo final alcanzado (simulación).');
        break;
    end
end

%% -------------------- Plots finales --------------------
figure('Name','Resultados','Color','w');
subplot(2,2,[1 3]);
hold on;
plot(Xlog(1,1:find(Xlog(1,:)~=0,1,'last')), Xlog(2,1:find(Xlog(2,:)~=0,1,'last')), ...
    'b-','LineWidth',1.5);
xlim([x_min x_max]);
ylim([y_min y_max]);
axis equal; grid on; xlabel('X [m]'); ylabel('Y [m]'); title('Trayectoria realizada');
%legend('Real');

subplot(2,2,2);
plot((1:length(Errlog))/50, Errlog(1,:)); xlabel('Tiempo [s]'); ylabel('eP [mm]');
title('Error posicion (mm)'); grid on;

subplot(2,2,4);
plot((1:length(rpm_log))/50, rpm_log'); xlabel('Tiempo [s]'); ylabel('RPM'); 
title('RPM ruedas'); grid on;

sgtitle('Resumen simulación');


% -----------------------------------------------------------------------------
% Simulación - Trayectorias con obstáculos y mapa para plataforma omnidireccional 
% -----------------------------------------------------------------------------
% Propósito
%   Simular y visualizar el seguimiento de trayectorias con planeación basada en
%   obstáculos, meta y posición actual de la plataforma omnidireccional de 4 
%   ruedas OmniRobotCC. Incluye:
%     - Planeación: D* basado en obstáculos, meta y posición inicial predefinida.
%     - Controlador PI en velocidades del cuerpo + modelo simple de motores.
%     - Animación en tiempo real y gráficos de resumen.
%
% Uso rápido
%   1) Ajuste parámetros en el bloque "Inicio y meta" (start_world y
%   goal_world) "Definir obstáculos" (obstacles) y "Parámetros de la 
%   simulación" (Kp, Ki).
%   2) Ejecute el script. 
%   3) Observe animación y gráficos finales; revise variables de salida.
%
% Variables importantes (salida / entradas)
%   Entradas configurables (en el script):
%     - start_world : [x y] posición inicial simulada (m)
%     - goal_world  : [x y] meta (m)
%     - obstacles   : [x y; x2 y2] posición obstáculos (m)
%     - dt, Tsim, Kp, Ki, rpm_max, lookahead_dist
%   Salidas / logs:
%     - p       : path coarse en metros (Nx2)
%     - traj_x, traj_y : trayectoria suavizada (m)
%     - p_mm    : trayectoria en mm (Nx2) (formato usado en lazo de control)
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
disp("----------Simulación Seguimiento de Trayectorias con Obstáculos con Control " + ...
    "FeedForward+PI Robot Omnidireccional----------");
%% -------------------- Parámetros del mundo / mapa --------------------
map_w = 4.0;   % ancho en metros (X)
map_h = 5.0;   % alto en metros (Y)

% Origen del mundo: colocamos (0,0) en el centro del mapa para la simulaci?n
x_min = -map_w/2; x_max = map_w/2;
y_min = -map_h/2; y_max = map_h/2;

%% ----- Definir obstáculos (centros en m) y visualizar mapa -----
% Simulación de configuraciones de obstáculos
obstacles = [  0.6,  0.8;   % [x, y]
              -0.8, -0.3;
               0.0, -1.0;
              -1.2,  1.2;
               1.1,  0.0;
               1.5, 1.0];

% Generar mapa de ocupación
map=genmap(obstacles);
figure(1);
imshow(map);
title('Mapa actual Robotat', ...
    'FontSize', 14, 'Interpreter', 'latex');
xlabel('$x$ (en cm)', 'FontSize', 14, 'Interpreter', 'latex');
ylabel('$y$ (en cm)', 'FontSize', 14, 'Interpreter', 'latex');

%legend();
axis on;

%% -------------------- Inicio y meta (en metros) --------------------
start_world = [0.0, 0.0];   % [x,y] 
goal_world  = [ 1.50,  2.0];   % [x,y]

% Convertir a celdas
world2grid = @(p) [ round( (p(1)-x_min)*100 ), round( (p(2)-y_min)*100 ) ]; % -> (row,col)

start_rc = world2grid(start_world); %Convertir a celdas el inicio
goal_rc  =  world2grid(goal_world); %Convertir a celdas la meta


%% -------------------- Planificador D* (grid) --------------------
% Implementación Dstar
dx = Dstar(map);

% Establecer meta
dx.plan(goal_rc);

% Consultar ruta desde el inicio 
path_rc=dx.query(start_rc, 'animate');

input("Presiona enter para continuar")


% Convertir ruta a coordenadas de mundo (m)
grid2world = @(rc) [ (-x_max*100+rc(1))/100, (-y_max*100+rc(2))/100 ]; % -> (x,y)
p = zeros(size(path_rc,1), 2);
for k=1:size(path_rc,1)
    p(k,:) = grid2world(path_rc(k,:)); % [x y]
end

% Suavizar ruta con spline (que no es tan necesario)
t_pts = linspace(0,1,size(p,1));
ppx = spline(t_pts, p(:,1)');
ppy = spline(t_pts, p(:,2)');

n_samples = max(200, round( norm(goal_world-start_world)/0.01 ));
ts = linspace(0,1,n_samples);
traj_x = ppval(ppx, ts);
traj_y = ppval(ppy, ts);

% Construir path en el formato p como columnas [x(mm), y(mm)]
p_mm = [traj_x(:)*1000, traj_y(:)*1000]; % mm

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
rotate_policy = 'never'; % opciones: 'only_at_goal', 'late', 'never', 'always'
rotate_fraction = 0.7; % si policy 'late', permitir rotar cuando idx_goal >= rotate_fraction*n_pts

% Estado inicial
state = [start_world(1); start_world(2); 0]; % [x;y;theta]

% Control (pure pursuit + PID exponencial/PI en velocidad cuerpo)
control = "PI"; 

% Pure pursuit param
lookahead_dist = 190/2; % mm
n_pts = size(p_mm,1);

% PID en velocidad cuerpo (PI) - ganancias iniciales (simulación)
kp_phi = 3.2; kp_x = 3.2; kp_y = 3.2;
ki_phi = 0.5; ki_x = 0.5; ki_y = 0.5;
Kp = diag([kp_x, kp_y, kp_phi]);
Ki = diag([ki_x, ki_y, ki_phi]);

%Matriz de error
Ierr = zeros(3,1);
I_max = [1.0; 0.5; 0.5]; I_min = -I_max;

% Para visualización del robot
robot_w = 0.19; robot_h = 0.19; % m
base_rect = [-robot_h/2, -robot_w/2;
              robot_h/2, -robot_w/2;
              robot_h/2,  robot_w/2;
             -robot_h/2,  robot_w/2]';

%% Figura del mapa-las trayectorias y la animación de la plataforma móvil
fig_map = figure('Name','Mapa y planificación','Color','w');
subplot(1,2,1);
imagesc(linspace(x_min,x_max), linspace(y_min,y_max), map); axis xy; axis equal;
hold on; colormap(gray); title('Mapa (0=libre,1=ocupado)');
plot(start_world(1), start_world(2), 'go', 'MarkerFaceColor','g');
plot(goal_world(1), goal_world(2), 'rx', 'MarkerSize',10, 'LineWidth',2);
plot(p(:,1), p(:,2), 'b-', 'LineWidth',1.2); % ruta coarsed antes de spline
plot(traj_x, traj_y, 'r--', 'LineWidth',1.0); % ruta suavizada
legend("Inicio", "Meta","Ruta inicial", "Ruta Suavizada");
xlabel('X [m]'); ylabel('Y [m]');

subplot(1,2,2);
axis equal; axis([-map_w/2 map_w/2 -map_h/2 map_h/2]); grid on; hold on;
xlabel('X [m]'); ylabel('Y [m]'); title('Animación Robot');

hPatch = patch('XData',[], 'YData',[], 'FaceColor',[.6 .8 .9], 'EdgeColor','k');
hTrail = animatedline('Color','r','LineWidth',1.5);
hMeta = plot(traj_x(1), traj_y(1),'*r');
% visual heading (flecha que muestra el eje X del robot)
heading_len = 0.19/2; % longitud de la flecha en metros 
hHeading = quiver(0,0,0,0,0, 'LineWidth', 2, 'MaxHeadSize', 0.6, ...
                  'Color', [0 0 0.6], 'AutoScale', 'off'); % inicializa vacío


hTime = text(-map_w/2+0.05, map_h/2-0.15, '', 'FontSize', 10, 'FontWeight','bold');
hVelRuedas = text(map_w/2-0.6, map_h/2-0.3, '', 'FontSize',9, 'FontName','Courier New');

%% Animación de la plataforma

% Variables auxiliar
t = 0; kidx = 1; idx_goal = 1;

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


% loop principal (pure-pursuit + control)
for k = 1:length(time)
    t = time(k);
    % ------------------ elegir punto meta (lookahead) ------------------
    while idx_goal < n_pts
        pm_tmp = p_mm(idx_goal,:); % [x_mm y_mm]
        pm_tmp_m = pm_tmp/1000;
        dx = pm_tmp_m(1) - state(1);
        dy = pm_tmp_m(2) - state(2);
        if hypot(dx*1000, dy*1000) >= lookahead_dist
            break;
        end
        idx_goal = idx_goal + 1;
    end

    if idx_goal >= n_pts && eP < lookahead_dist/2 && norm(eO)<deg2rad(15)
        % ruta consumida -> parada
        u_rpm_cmd = [0;0;0;0];
        % actualizar logs finales y salir
        rpm_cmd_log(:,k) = u_rpm_cmd;
        rpm_log(:,k) = u_rpm_meas;
        Xlog(:,k) = state;
        Errlog(:,k) = [eP; 0; 0];
        disp('Ruta consumida. Fin simulación.');
        break;
    end

    block_theta = false;
    switch rotate_policy
        case 'only_at_goal'
            if ~(idx_goal == n_pts && eP < 1.5*lookahead_dist)
                block_theta = true;
            end
        case 'late'
            if idx_goal < max(1, round(rotate_fraction * n_pts))
                block_theta = true;
            end
        case 'never'
            block_theta = true;
        otherwise
            block_theta = false;
    end

    % punto meta en metros
    pm = p_mm(idx_goal,:)/1000; xg = pm(1); yg = pm(2);

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
    %u_rpm_meas
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
    set(hVelRuedas, 'String', sprintf(['v=%.3f m/s  w=%.3f rad/s\nRPM cmd:' ...
        ' %6.0f %6.0f %6.0f %6.0f\nRPM real:%6.0f %6.0f %6.0f %6.0f'],...
        v_display, w_display, u_rpm_cmd(1),u_rpm_cmd(2),u_rpm_cmd(3),u_rpm_cmd(4),...
        u_rpm_meas(1),u_rpm_meas(2),u_rpm_meas(3),u_rpm_meas(4)));

    drawnow limitrate;
    pause(dt);

    % Logging
    Xlog(:,k) = state;
    rpm_log(:,k) = u_rpm_meas;
    Errlog(:,k) = [eP; eO; 0];

    
end

%% -------------------- Plots finales --------------------
figure('Name','Resultados','Color','w');
subplot(2,2,[1 3]);
plot(traj_x, traj_y,'r--','LineWidth',1); hold on;
plot(Xlog(1,1:find(Xlog(1,:)~=0,1,'last')), Xlog(2,1:find(Xlog(2,:)~=0,1,'last')),'b-', ...
    'LineWidth',1.5);
axis equal; grid on; xlabel('X [m]'); ylabel('Y [m]'); title('Trayectoria deseada vs real');
xlim([x_min x_max]);
ylim([y_min y_max]);
legend('Deseada','Real');



subplot(2,2,[2 4]);
plot((1:length(rpm_log))/50, rpm_log'); xlabel('Tiempo [s]'); ylabel('RPM'); title('RPM ruedas'); 
grid on;


figure('Name','Resultados_2','Color','w');
plot((1:length(Errlog))/50, Errlog(1,:)); xlabel('Tiempo [s]'); ylabel('eP [mm]'); 
title('Error posicion (mm)'); 
grid on;
sgtitle('Resumen simulación');


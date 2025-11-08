% ----------------------------------------------------------------------------- 
% Simulación - Trayectorias predefinidas para plataforma omnidireccional 
% ----------------------------------------------------------------------------- 
% Propósito 
% Simular y visualizar el seguimiento de trayectorias predefinidas por la 
% plataforma omnidireccional de 4 ruedas OmniRobotCC. Incluye: 
% - Patrones: combinación (vuelve al centro), círculo, figura-8. 
% - Fase inicial: mover desde posición inicial simulada al centro [0,0], 
% y ejecutar la maniobra desde el centro. 
% - Controlador PI en velocidades del cuerpo + modelo simple de motores. 
% - Animación en tiempo real y gráficos de resumen. 
% % Uso rápido 
% 1) Ajuste parámetros en el bloque "Parámetros del mundo / área" y 
% "Parámetros de simulación" (start_world_init, len, circle_radius, dt, Kp, Ki). 
% 2) Ejecute el script. Seleccione la maniobra en el menú. 
% 3) Observe animación y gráficos finales; revise variables de salida. 
% % Variables importantes (salida / entradas) 
% Entradas configurables (en el script): 
% - start_world_init : [x y] posición inicial simulada (m) 
% - center_world : [0 0] centro del Robotat (m) 
% - len, circle_radius, scale, npt_per_seg, n_circle 
% - dt, Tsim, Kp, Ki, rpm_max, lookahead_dist 
% Salidas / logs: 
% - p : path coarse en metros (Nx2) 
% - traj_x, traj_y : trayectoria suavizada (m) 
% - p_mm : trayectoria en mm (Nx2) (formato usado en lazo de control) 
% - Xlog : poses simuladas durante la ejecución 
% - rpm_log : rpm reales simulados por rueda 
% - rpm_cmd_log : rpm comandos enviados al modelo 
% - Errlog : errores registrados (posición, yaw) 
% % Comportamiento y supuestos 
% - No controla hardware: es una simulación. Para integrar hardware, sustituir 
% modelos por llamadas robot_* y verificar conversiones de unidades y ángulos. 
% - Ángulos: todas las operaciones trigonométricas usan radianes. 
% - La trayectoria siempre se construye para comenzar en el centro. Si el robot 
% no está en el centro, el script genera un tramo previo lineal hacia éste. 
% % Tips rápidos de uso / debugging 
% - Si la trayectoria parece desplazada, verifique start_world_init y center_world. 
% - Si el robot gira mal: compruebe rotate_policy y kp_phi/ki_phi. 
% - Para pruebas sin rotación, usar rotate_policy = 'never'. 
% - Para depurar control, reduzca Tsim y aumente dt (más rápido para iterar). 
% % Autores / versión 
% - Autor: Christian Campos 
% - Versión: 1.0 
% - Fecha: 2025-10-13
% -----------------------------------------------------------------------------
close all; clear; clc;
% Fuente similar a la versión original
set(groot, ...
    'DefaultAxesFontName',  'Times New Roman', ...
    'DefaultTextFontName',  'Times New Roman', ...
    'DefaultAxesFontSize',  12, ...
    'DefaultTextFontSize',  12, ...
    'DefaultLegendFontName','Times New Roman', ...
    'DefaultLegendFontSize', 10);

disp("Simulación");

%% -------------------- Parámetros del mundo / área --------------------
map_w = 4.0;   % ancho en metros (X)
map_h = 5.0;   % alto en metros (Y)
x_min = -map_w/2; x_max = map_w/2;
y_min = -map_h/2; y_max = map_h/2;

% Posición inicial simulada del robot (puedes cambiarla)
start_world_init = [0.3, 1]; % posición inicial del robot en el mundo (m)
center_world = [0.3, 1];     % centro del Robotat (m) -> inicio de la maniobra

% Parámetros generales trayectoria (puedes cambiar traj_choice: 1,2,3)
traj_choice = 3;  % 1=Combinación, 2=Círculo, 3=Figure-8

size_scale = 1.0;          % factor global de tamaño
len = 0.6 * size_scale;    % tamaño característico (m)
npt_per_seg = 60;
circle_radius = 0.5 * size_scale;
n_circle = 300;

% 1) Tramo inicial lineal al centro
start_world = start_world_init;
dist_to_center = norm(start_world - center_world);
if dist_to_center < 1e-3
    path_to_center = center_world;
else
    n_move = max(round(dist_to_center / 0.01), 10);
    t_move = linspace(0,1,n_move)';
    path_to_center = (1-t_move)*start_world + t_move*center_world;
end

% 2) Generar patrón local
switch traj_choice
    case 1
        d = len;
        verts = [0, d; d, d; 0, 0; d, -d; -d, -d; 0,0];
        pts = [];
        for i = 1:size(verts,1)-1
            pA = verts(i,:); pB = verts(i+1,:);
            seg = [linspace(pA(1), pB(1), npt_per_seg)', linspace(pA(2), pB(2), npt_per_seg)'];
            pts = [pts; seg];
        end
        backseg = [linspace(verts(end,1), 0, npt_per_seg)', linspace(verts(end,2), 0, ...
            npt_per_seg)'];
        local_pattern = [pts; backseg];
    case 2
        t = linspace(0,2*pi,n_circle)';
        local_circle = [circle_radius*cos(t), circle_radius*sin(t)];
        ramp = [linspace(0, local_circle(1,1), npt_per_seg)', linspace(0, local_circle(1,2), ...
            npt_per_seg)'];
        local_pattern = [ramp; local_circle; flipud(ramp)];
    case 3
        t = linspace(0,2*pi,n_circle)';
        a = circle_radius * 0.8;
        local8 = [ a*sin(t), a*sin(t).*cos(t) ];
        ramp = [ linspace(0, local8(1,1), npt_per_seg)', linspace(0, local8(1,2), npt_per_seg)'];
        local_pattern = [ramp; local8; flipud(ramp)];
    otherwise
        t = linspace(0,2*pi,n_circle)';
        local_pattern = [circle_radius*cos(t), circle_radius*sin(t)];
        ramp = [ linspace(0, local_pattern(1,1), npt_per_seg)', linspace(0, local_pattern(1,2), ...
            npt_per_seg)'];
        local_pattern = [ramp; local_pattern; flipud(ramp)];
end

% % 3) Pasar a coordenadas del mundoro y concatenar
pattern_world = local_pattern + repmat(center_world, size(local_pattern,1), 1);
if isequal(size(path_to_center,1),1) && all(path_to_center == center_world)
    p = [center_world; pattern_world];
else
    p = [path_to_center; pattern_world];
end

% 4) Suavizado spline
t_pts = linspace(0,1,size(p,1));
ppx = spline(t_pts, p(:,1)');
ppy = spline(t_pts, p(:,2)');

n_samples = max(200, round( norm(p(end,:)-p(1,:))/0.01 * 10 ));
ts = linspace(0,1,n_samples);
traj_x = ppval(ppx, ts);
traj_y = ppval(ppy, ts);

% p_mm en mm (formato requerido por el controlador)
p_mm = [traj_x(:)*1000, traj_y(:)*1000]; % mm

fprintf("Trayectoria generada: coarse %d puntos, suavizada %d muestras\n",size(p,1), numel(traj_x));
pause(0.1);

%% -------------------- Parámetros físicos del robot --------------------
w = ((46.8+82.3)/2)/1000;   % m
l = ((74.65+32.35)/2)/1000; % m
r = 48/1000;                % radio rueda [m]
rpm_max = 80;    % rpm

H0 = [ -l - w, 1, -1;
        l + w, 1,  1;
        l + w, 1, -1;
       -l - w, 1,  1 ];
H0_pinv = pinv(H0);
v_max = max(abs(r*H0_pinv*(2*(pi/60)*[rpm_max; rpm_max; rpm_max; rpm_max])));
w_max = max(abs(r*H0_pinv*(2*(pi/60)*[rpm_max; -rpm_max; -rpm_max; rpm_max])));

%% Parámetros de simulación y control
dt = 0.1;        % s
Tsim = 300;      % s (límite)
time = 0:dt:Tsim;

rotate_policy = 'only_at_goal'; % política por defecto
lookahead_dist = 190/2; % mm
theta_final=90;
theta_final=wrapToPi(deg2rad(theta_final));

% Ganancias (valores similares a la versión hardware)
kp_phi = 0.8; kp_x = 1.6; kp_y = 1.6;
ki_phi = 0.15; ki_x = 0.15; ki_y = 0.15;
Kp = diag([kp_x, kp_y, kp_phi])*100;
Ki = diag([ki_x, ki_y, ki_phi])*100;

Ierr = zeros(3,1); I_max = [1.0;0.5;0.5]; I_min = -I_max;

% visualización robot
robot_w = 0.19; robot_h = 0.19;
base_rect = [-robot_h/2, -robot_w/2;
              robot_h/2, -robot_w/2;
              robot_h/2,  robot_w/2;
             -robot_h/2,  robot_w/2]';

%% -------------------- Map sintético (vacío) --------------------
% Crear un mapa vacío de 0/1 con la misma convención que usa la versión original.
% grid resolution: 100 cells/m (coincide con world2grid en el otro script).
nx = round(map_w*100); % celdas en X
ny = round(map_h*100); % celdas en Y
map = zeros(ny, nx);   % 0 = libre, 1 = ocupado
% (Si quieres añadir obstáculos, marca '1' en las celdas correspondientes)

%% -------------------- Figura del mapa + animación (fig_map) --------------------
fig_map = figure('Name','Mapa y planificación','Color','w', 'Position',[2 2 900 600]);
subplot(1,2,1);
imagesc(linspace(x_min,x_max,nx), linspace(y_min,y_max,ny), map'); axis xy; axis equal;
hold on; colormap(gray); title('Mapa (0=libre,1=ocupado)');
plot(start_world(1), start_world(2), 'go', 'MarkerFaceColor','g', 'DisplayName','Inicio');
% usar como "meta" el primer punto de la trayectoria
goal_world = p(end,:);
plot(goal_world(1), goal_world(2), 'rx', 'MarkerSize',10, 'LineWidth',2, 'DisplayName','Meta');
plot(p(:,1), p(:,2), 'b.', 'LineWidth',1.2, 'DisplayName','Ruta initial');
plot(traj_x, traj_y, 'r--', 'LineWidth',1.0, 'DisplayName','Ruta Suavizada');
legend('Location','best');
xlabel('X [m]'); ylabel('Y [m]');

subplot(1,2,2);
axis equal; axis([-map_w/2 map_w/2 -map_h/2 map_h/2]); grid on; hold on;
xlabel('X [m]'); ylabel('Y [m]'); title('Animación Robot');

hPatch = patch('XData',[], 'YData',[], 'FaceColor',[.6 .8 .9], 'EdgeColor','k');
hTrail = animatedline('Color','r','LineWidth',1.5);
hMeta = plot(traj_x(1), traj_y(1),'*r');
heading_len = 0.19/2;
hHeading = quiver(0,0,0,0,0, 'LineWidth', 2, 'MaxHeadSize', 0.6, ...
                  'Color', [0 0 0.6], 'AutoScale', 'off');
hTime = text(-map_w/2+0.05, map_h/2-0.15, '', 'FontSize', 10, 'FontWeight','bold');
hVelRuedas = text(-map_w/2+0.05, map_h/2-0.45, '', 'FontSize', 9);

drawnow;

%% -------------------- Inicializaciones para simulación --------------------
state = [start_world(1); start_world(2); pi]; % [x;y;theta]
idx_goal = 1;
n_pts = size(p_mm,1);

% Motor model params (simple)
rng(1);
u_rpm_meas = zeros(4,1);
tau_motor = 0.12;
motor_gain = 0.98 + 0.08*randn(4,1);
motor_bias = 0.5*randn(4,1);
motor_noise_std = 2.0;
r_wheels = r ;

% Logs (pre-asignar similar a versión original)
max_iter = ceil(300/dt);
Xlog = zeros(6, max_iter);        % [x;y;theta; idx_goal; xg; yg] -> filas
Errlog = zeros(1, max_iter);      % eP [mm]
CTE_log = zeros(1, max_iter);     % CTE abs [mm]
CTE_signed_log = zeros(1, max_iter);
rpm_log = zeros(4, max_iter);     % rpm ruedas
rpm_cmd_log = zeros(4, max_iter); % rpm comandos
time_log = zeros(1, max_iter);

log_idx = 0;

%% -------------------- Bucle de simulación (modelo de motores) --------------------
for k = 1:length(time)
    t = time(k);
    % seleccionar objetivo lookahead
    while idx_goal < n_pts
        pm_tmp = p_mm(idx_goal,:);
        pm_tmp_m = pm_tmp/1000;
        dx = pm_tmp_m(1) - state(1);
        dy = pm_tmp_m(2) - state(2);
        if hypot(dx*1000, dy*1000) >= lookahead_dist
            break;
        end
        idx_goal = idx_goal + 1;
    end

    % punto meta (m)
    pm = p_mm(idx_goal,:)/1000; xg = pm(1); yg = pm(2);

    % pose actual
    x = state(1); y = state(2); th = state(3);

    % errores
    e = [xg - x; yg - y];
    eP = norm(e)*1000; % mm

    % política de rotación sencilla (comportamiento parecido a original)
    switch rotate_policy
        case 'only_at_goal'
            block_theta = ~(idx_goal == n_pts && eP < 1.5*lookahead_dist);
        otherwise
            block_theta = true;
    end

    if block_theta
        thetag = 0;
    else
        thetag = theta_final;
    end
    eO = wrapToPi(thetag - th);

    err = [ e(1); e(2); eO ];
    Ierr_cand = Ierr + err * dt;
    Ierr_cand = max(min(Ierr_cand, I_max), I_min);

    % velocidad deseada entre puntos
    if idx_goal < n_pts
        pm_next = p_mm(idx_goal+1,:) / 1000;
        v_des = (pm_next - pm) / dt;
        if block_theta
            w_des = 0.2 * eO;
        else
            theta_current = thetag;
            theta_next = atan2(pm_next(2)-pm(2), pm_next(1)-pm(1));
            w_des = wrapToPi(theta_next - theta_current) / dt;
        end
        % scale velocities
        if norm(v_des) > v_max
            vel_scale = v_max / norm(v_des); v_des = vel_scale * v_des;
        end
        if abs(w_des) > w_max
            w_scale = abs(w_max / w_des); w_des = w_scale * w_des;
        end
    else
        v_des = [0 0]; w_des = 0;
    end

    % feedforward + PI
    qd_dot_world = [v_des(1); v_des(2); w_des];
    qdot_world = qd_dot_world + Kp * err + Ki * Ierr_cand;

    % mundo -> cuerpo
    R_th = [cos(th), -sin(th); sin(th), cos(th)];
    v_body = R_th' * qdot_world(1:2);
    omega = qdot_world(3);
    Vb_cmd = [omega; v_body(1); v_body(2)];

    % convertir a rpm
    u_rad_s_cand = (1/r) * H0 * Vb_cmd;
    u_rpm_cand = u_rad_s_cand * 60 / (2*pi);

    if any(abs(u_rpm_cand) > rpm_max)
        scale_vel = rpm_max / max(abs(u_rpm_cand));
        u_rpm_cmd = u_rpm_cand * scale_vel;
        u_rad_s = u_rpm_cmd * 2*pi / 60;
        Vb_real_from_cmd = r * H0_pinv * u_rad_s;
        % no actualizar integrador
    else
        u_rpm_cmd = u_rpm_cand;
        u_rad_s = u_rpm_cmd * 2*pi / 60;
        Vb_real_from_cmd = Vb_cmd;
        Ierr = Ierr_cand;
    end
    rpm_cmd_log(:,k) = u_rpm_cmd;

    % motor model (1st order)
    target_rpm = motor_gain .* u_rpm_cmd + motor_bias;
    noise = motor_noise_std * sqrt(dt) .* randn(4,1);
    u_rpm_meas = u_rpm_meas + (dt / tau_motor) .* (target_rpm - u_rpm_meas) + noise;
    deadband = 0.4; u_rpm_meas(abs(u_rpm_meas) < deadband) = 0;
    u_rad_s_meas = u_rpm_meas * 2*pi / 60;
    Vb_real = H0_pinv * (r_wheels .* u_rad_s_meas);

    omega_bz = Vb_real(1);
    v_bx = Vb_real(2);
    v_by = Vb_real(3);
    vx_world = v_bx*cos(th) - v_by*sin(th);
    vy_world = v_bx*sin(th) + v_by*cos(th);

    state(1) = state(1) + vx_world*dt;
    state(2) = state(2) + vy_world*dt;
    state(3) = wrapToPi(state(3) + omega_bz*dt);

    % Visualización
    pos = [cos(state(3)), -sin(state(3)); sin(state(3)), cos(state(3))] * base_rect + state(1:2);
    set(hPatch, 'XData', pos(1,:), 'YData', pos(2,:));
    addpoints(hTrail, state(1), state(2));
    set(hMeta, 'XData', xg, 'YData', yg);

    hx = state(1); hy = state(2);
    ux = heading_len * cos(state(3)); uy = heading_len * sin(state(3));
    set(hHeading, 'XData', hx, 'YData', hy, 'UData', ux, 'VData', uy);

    v_display = norm([Vb_real(2), Vb_real(3)]);
    w_display = Vb_real(1);
    set(hTime, 'String', sprintf('Tiempo: %.2f s | eP=%.0f mm | eO=%.2f deg', t, eP, rad2deg(eO)));
    set(hVelRuedas, 'String', sprintf(['v=%.3f m/s  w=%.3f rad/s\nRPM cmd: ' ...
        '%6.0f %6.0f %6.0f %6.0f\nRPM real:%6.0f %6.0f %6.0f %6.0f'],...
        v_display, w_display, u_rpm_cmd(1),u_rpm_cmd(2),u_rpm_cmd(3),u_rpm_cmd(4),...
        u_rpm_meas(1),u_rpm_meas(2),u_rpm_meas(3),u_rpm_meas(4)));
    drawnow limitrate;
    pause(dt);

    % Logging (formato compatible con original)
    log_idx = log_idx + 1;
    Xlog(:,log_idx) = [state; idx_goal; xg; yg];   % 6 x N
    Errlog(1,log_idx) = eP;                       % mm
    rpm_log(:,log_idx) = u_rpm_meas;              % rpm measured
    rpm_cmd_log(:,log_idx) = u_rpm_cmd;
    time_log(log_idx) = t;
    % CTE respecto al segmento objetivo (en mm)
    P_mm = state(1:2)'*1000;
    if idx_goal < n_pts
        A = p_mm(idx_goal, :); B = p_mm(min(idx_goal+1, n_pts), :);
    else
        A = p_mm(max(1, idx_goal-1), :); B = p_mm(idx_goal, :);
    end
    AB = B - A; AP = P_mm - A; denom = dot(AB,AB);
    if denom == 0
        Q = A;
    else
        tproj = dot(AP,AB)/denom; tcl = max(0,min(1,tproj)); Q = A + tcl*AB;
    end
    vecPQ = P_mm - Q; cte_abs_mm = norm(vecPQ);
    if norm(AB) < eps
        cte_signed_mm = 0;
    else
        cte_signed_mm = (AB(1)*AP(2) - AB(2)*AP(1)) / norm(AB);
    end
    CTE_log(log_idx) = cte_abs_mm;
    CTE_signed_log(log_idx) = cte_signed_mm;

    % condición de término (ruta consumida)
    if idx_goal >= n_pts && eP < lookahead_dist/2 && abs(eO) < deg2rad(15)
        disp('Ruta consumida. Fin simulación.');
        break;
    end
end % for

% recortar logs al tamaño real
if log_idx == 0
    warning('No se registraron datos (log_idx=0). Termina ejecución.');
    return;
end
Xlog = Xlog(:,1:log_idx);
Errlog = Errlog(1:log_idx);
rpm_log = rpm_log(:,1:log_idx);
rpm_cmd_log = rpm_cmd_log(:,1:log_idx);
time_log = time_log(1:log_idx);
CTE_log = CTE_log(1:log_idx);
CTE_signed_log = CTE_signed_log(1:log_idx);

%% -------------------- Plots finales (compatibles con la versión original) ----
figure('Name','Resultados_1','Color','w', ...
   'DefaultAxesFontName','Times New Roman', ...
   'DefaultTextFontName','Times New Roman', ...
   'DefaultAxesFontSize',12, ...
   'DefaultTextFontSize',12);

% 1) Trayectoria deseada vs real
subplot(2,2,[1 3]);
plot(traj_x, traj_y,'r--','LineWidth',1); hold on;
plot(Xlog(1,:), Xlog(2,:),'b-','LineWidth',1.5);
axis equal; grid on; xlabel('X [m]'); ylabel('Y [m]'); title('Trayectoria deseada vs real');
xlim([x_min x_max]); ylim([y_min y_max]);
legend('Deseada','Real');

% 3) RPM plot
tvec = time_log;
subplot(2,2,[2 4]);
plot(tvec, rpm_log');
xlabel('Tiempo [s]'); ylabel('RPM'); title('RPM ruedas (real)'); grid on;


sgtitle('Resumen simulación - Trayectorias predefinidas');

% 2) Error posición (eP)
figure('Name','Resultados_eP','Color','w', ...
   'DefaultAxesFontName','Times New Roman', ...
   'DefaultTextFontName','Times New Roman', ...
   'DefaultAxesFontSize',12, ...
   'DefaultTextFontSize',12);

hold on;
plot(tvec, Errlog, 'b', 'LineWidth', 1.2);
yline(lookahead_dist, '-.', 'LineWidth', 1.5, 'Color', [95,164,56]/255);
RMSE_pos_mm = sqrt(mean((Errlog-lookahead_dist).^2));
yline(RMSE_pos_mm + lookahead_dist, '--', 'LineWidth', 1.5, 'Color', 'r');
RMSE_cte_mm = sqrt(mean((CTE_log-lookahead_dist).^2));
yline(RMSE_cte_mm + lookahead_dist, '--', 'LineWidth', 1.5, 'Color', 'm');
[max_eP_mm, idx_max] = max(Errlog);
t_max = tvec(idx_max);
plot(t_max, max_eP_mm, 'ro', 'MarkerFaceColor','r', 'MarkerSize',8);
%text(t_max, max_eP_mm, sprintf('  max eP = %.1f mm', max_eP_mm), 'Color','r','FontWeight','bold');
xlabel('Tiempo [s]'); ylabel('eP [mm]'); title('Error posición (mm) con límites'); grid on; box on;
legend({'eP (mm)', 'Lookahead', 'RMSE_{pos}', 'RMSE_{cte}','Máx eP'}, 'Location', 'best');



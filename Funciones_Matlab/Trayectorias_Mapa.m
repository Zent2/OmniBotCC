% =============================================================================
%   Trayectorias con obstáculos y mapa para plataforma omnidireccional
%   (OmniBotCC)
% -----------------------------------------------------------------------------
% Propósito
%   Script para planificación y ejecución de trayectorias en el OmniBotCC 
%   usando:
%     - Lectura de marcadores desde Robotat para generar un mapa de obstáculos
%       (función genmap).
%     - Planificación de ruta en grilla con D* (Dstar).
%     - Suavizado de la ruta (spline) y seguimiento en hardware con un
%       controlador PI+FeedForward en velocidades del cuerpo.
%     - Animación en tiempo real (patch + trail) y registro de datos.
%
% Flujo general / Resumen
%   1) Conexión a Robotat (visualización de marcadores opcional) y al robot.
%   2) Lectura de marcadores seleccionados -> lista de obstáculos -> genmap.
%   3) Selección de inicio (pose actual) y meta (centro / marcador / coordenada).
%   4) Planificación D* sobre la grilla, obtención de ruta y suavizado (spline).
%   5) Bucle de control en hardware:
%        - Pure pursuit (lookahead) para seleccionar punto objetivo.
%        - Control PI + FeedForward para generar velocidades en mundo.
%        - Conversión a velocidades de rueda (rad/s -> RPM), saturación y envío.
%        - Logging, animación y emergencia (STOP con tecla ENTER).
%   6) Mostrar resumen y gráficas finales.
%
% Variables importantes (entrada / salida)
%   Entradas/configurables:
%     - ids                 : vector de IDs de marcadores en Robotat
%     - id                  : ID del robot en Robotat (pose)
%     - Offsets.mat         : archivo con offsets (offsets_mm, offsets_m)
%     - rotate_option       : política de rotación ('only_at_goal','late','never', 'always')
%     - kp_x, kp_y, kp_phi, ki_x, ki_y, ki_phi : ganancias PI
%     - lookahead_dist, tol_mm, rpm_max, dt, Tsim, etc.
%   Salidas / logs:
%     - map        : mapa de ocupación generado por genmap (matriz)
%     - p          : ruta coarse en metros (Nx2)
%     - traj_x,y   : trayectoria suavizada (m)
%     - p_mm       : trayectoria en mm (Nx2) — usada por el lazo de control
%     - Xlog       : registro de poses [x;y;theta] durante la ejecución
%     - Errlog     : registro de errores (mm)
%     - rpm_log    : RPM por rueda registrados
%     - time_log   : tiempos del log
%
% Comportamiento y supuestos
%   - Este script está pensado para ejecutarse con hardware (Robotat + robot).
%     Asegúrese de que las funciones robotat_* y robot_* estén disponibles.
%     Así como las funciones genmap, q2rot y q2eul.
%   - Unidades:
%       * coordenadas del mapa y planificación: metros (m)
%       * rutas internas para control: milímetros (mm) en p_mm
%       * yaw/ángulos: Offsets suelen guardarse en grados; el script convierte a
%         radianes (deg2rad) antes de usar cos/sin/rotaciones.
%   - Seguridad: incluye listener para parada de emergencia (ENTER) y manejo
%     de fallos en robot_set_velocities / robot_start / robot_stop.
%   - La clase Dstar debe estar en el path y funcionar sobre la representación
%     de grilla que genera genmap.
%
% Tips rápidos / recomendaciones
%   - Verifique Offsets.mat y confirme que offsets_m(4) esté en guardado en grados 
%     el script espera grados y aplica deg2rad donde procede.
%   - Si la conexión falla, revise que Robotat y el robot estén en la misma red
%     y que las APIs/servicios estén activos.
%   - Ajuste kp/ki y lookahead_dist para mejorar seguimiento.
%
% Registro y reproducibilidad
%   - Logs relevantes: Xlog, Errlog, rpm_log, time_log — guarde en .mat si
%     se desea post-procesado.
%
% Autor / versión
%   - Autor: Christian Campos
%   - Versión: 1.0 
%   - Fecha: 2025-10-13
% =============================================================================
close all; clc;
% ------------------------- Fuente Times New Roman 12 -------------------------
set(groot, ...
    'DefaultAxesFontName',  'Times New Roman', ...
    'DefaultTextFontName',  'Times New Roman', ...
    'DefaultAxesFontSize',  12, ...
    'DefaultTextFontSize',  12, ...
    'DefaultLegendFontName','Times New Roman', ...
    'DefaultLegendFontSize', 10);

%clear;
disp("----------Seguimiento de Trayectorias con Obstáculos con Control " + ...
    "FeedForward+PI Robot Omnidireccional----------");
%% --------------------------- Conexión Al Robotat ----------------------------
disp("Conectando al Robotat");

% Intentamos conectarnos al Robotat
try 
    robotat_disconnect(robotat); %Si ya existe la conexión la borramos
catch ME
    %fprintf("%s\n", ME.message);
end

% Ahora intentamos realizar la conexión
while true 
    try
        % Intentar conectar
        robotat = robotat_connect();
        fprintf("\nConexión exitosa\n");
        break; % salir del while si se conecta
    catch ME
        % Si falla la conexión
        error("No se pudo conectar al Robotat.\nError: %s", ME.message);
    end
end

%% ------------------ Conexión a la plataforma móvil --------------------------
disp("Conectando con el Robot");

timeout = 6;      % tiempo máximo de espera en segundos
t0 = tic;
% Intentamos conectarnos al Robot
try 
    robot_disconnect(); % Si ya existe la conexión la eliminamos
catch ME
    %fprintf("%s\n", ME.message);

end

% Ahora intentamos realizar la conexión
while true
    try
        % Intentar conectar
        robot_connect(); 
        fprintf("\nConexión exitosa en %.2f s\n", toc(t0));
        break; % salir del while si se conecta
    catch ME
        % Si falla la conexión
        if toc(t0) > timeout
            error("No se pudo conectar al Robot en %d segundos.\nError: %s", timeout, ME.message);
        end
        fprintf("."); % imprimir punto mientras intenta
        pause(0.5);   % espera antes de reintentar
    end
end
fprintf("\nADVERTENCIA: La conexión es UDP por lo que no está confirmado al 100% la " + ...
    "conexión, verifique que sí se están recibiendo los comandos\n");
clear t0;

%% ------------------ Visualización inicial en el robotat ---------------------
id=29; %ID del robot en el robotat
fprintf("\nVisualización del robot con ID: %d \n", id);

%Comprobar si se quiere o no visualizar el Robot en el Robotat.
while true
    visual = upper(input("¿Desea visualizar la posición del marcador en el Robotat? (Y/N): ", "s"));
    
    if visual == "Y"
        fprintf("Visualizando el Robot...\n");
        %Visualizar el robot 
        robotat_trvisualize(robotat,id);
        input("Presione enter para continuar:");
        break;
    elseif visual == "N"
        fprintf("Saltando visualización inicial\n");
        break;
    else
        fprintf("Valor incorrecto, debe ingresar Y o N\n");
    end
end

%% ------------------------ Parámetros del mundo/mapa -------------------------
map_w = 4.0;   % ancho en metros (X)
map_h = 5.0;   % alto en metros (Y)
% Origen del mundo: colocamos (0,0) en el centro del mapa para la simulación
x_min = -map_w/2; x_max = map_w/2;
y_min = -map_h/2; y_max = map_h/2;
walls=0.1; %Offsets de las paredes para que el Robot no tope con ellas

%% ----------------------- Parámetros fí­sicos del robot -----------------------
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
%Limites
v_max = max(abs(r*H0_pinv*(2*(pi/60)*[rpm_max; rpm_max; rpm_max; rpm_max]))); % [m/s] 
w_max=max(abs(r*H0_pinv*(2*(pi/60)*[rpm_max; -rpm_max; -rpm_max; rpm_max]))); % [rad/s]
dt = 0.1;        % s

%% ------------ Definir obstáculos (centros en m) y visualizar mapa -----------
disp("Definición de los obstáculos y visualización del mapa");

% Obstáculos iniciales
ids = [68 65 73 75 63 70 74 64 71 67 66 62 61 69 19 23];  % marcadores de obstáculos en Robotat

fprintf("Los marcadores definidos actualmente son: ");
fprintf("%d ", ids);
fprintf("\n");
add_more = upper(input("¿Desea añadir más? (Y/N): ", "s"));
while true    
    if add_more == "Y"
        new_ids = input("Ingrese los nuevos marcadores en formato [id1 id2 ...]: ");
        if isnumeric(new_ids) && ~isempty(new_ids)
            ids = [ids new_ids]; %#ok<AGROW>
            fprintf("Marcadores actualizados: ");
            fprintf("%d ", ids);
            fprintf("\n");
            break;
        else
            fprintf("Entrada inválida. Debe ser un vector de números.\n");
        end
    elseif add_more == "N"
        fprintf("Lista final de marcadores: ");
        fprintf("%d ", ids);
        fprintf("\n");
        break;
    else
        fprintf("Valor incorrecto, debe ingresar Y o N\n");
    end
end

disp("Lectura de los valores de los obstáculos");
% Inicialización del array de obstáculos
obstacles=[];
% Se recorre cada ID y se obtiene su posición con robotat_get_pose
for i = 1:length(ids)
    pm = robotat_get_pose(robotat, ids(i), 'eulxyz');
    
    % Validación por si no se puede obtener la posición
    if ~isempty(pm)
        % Se agrega la posición (x, y) al array de obstáculos
        obstacles(end+1, :) = pm(1:2);
    end
end

% Generar mapa de ocupación (0 libre, 1 ocupado)
disp("Visualización del Mapa");
map=genmap(obstacles);
figure(2);
imshow(map);
title('Mapa actual Robotat', ...
    'FontSize', 12, 'FontName', 'Times New Roman');
xlabel('$x$ (en cm)', 'FontSize', 12, 'FontName', 'Times New Roman');
ylabel('$y$ (en cm)', 'FontSize', 12, 'FontName', 'Times New Roman');

%legend();
axis on;


%% ------------------------- Inicio y meta (en metros) ------------------------
disp("Definiendo inicio (Robot) y meta:");

% Cargar offsets y obtener pose actual
load Offsets; 
robot_pos = robotat_get_pose(robotat, id, 'eulzyx');
offset = offsets_m;

xpos = (robot_pos(1) - offset(1)) * 1000; % mm
ypos = (robot_pos(2) - offset(2)) * 1000; % mm
yaw  = atan2d(sind(robot_pos(4) - offset(4)), cosd(robot_pos(4) - offset(4))); % °
th = deg2rad(yaw);
robot_pos = [xpos, ypos, offset(3), yaw];

fprintf("Posición Actual del Robot:\n X = %.2f mm\n Y = %.2f mm\n Z = %.2f mm\n Yaw = %.2f °\n", ...
        xpos, ypos, offset(3), yaw);

% Inicio del mundo (robot actual en m)
start_world = [xpos/1000, ypos/1000];

% Selección de meta
while true
    goal = input("¿Cuál será la meta? (1=Centro [0 0], 2=Marcador, 3=Coordenada fija): ");
    if goal == 1
        goal_world = [0, 0]; % Centro del Robotat
        disp("Meta establecida en el centro [0 0].");
        break;
    elseif goal == 2
        goal_id = input("Ingrese el ID del marcador de la meta: ");
        goal_pos = robotat_get_pose(robotat, goal_id, 'eulxyz');
        if isempty(goal_pos)
            warning("No se pudo obtener la posición del marcador %d. Intente de nuevo.", goal_id);
        else
            goal_world = goal_pos(1:2); % [x, y] en metros
            fprintf("Meta establecida en marcador %d: [%.3f, %.3f] m\n", goal_id, goal_world(1), ...
                goal_world(2));
            break;
        end
    elseif goal == 3
        while true
            goal_world = input("Ingrese la coordenada en formato [x y] (en metros): ");
            if isnumeric(goal_world) && numel(goal_world) == 2
                gx = goal_world(1); gy = goal_world(2);
                if gx >= x_min+walls && gx <= x_max-walls && gy >= y_min+walls && gy <= y_max-walls
                    fprintf("Meta establecida en coordenada fija: [%.3f, %.3f] m\n", gx, gy);
                    break;
                else
                    warning("Coordenada fuera de los límites del mapa (X:[%.1f, %.1f], " + ...
                        "Y:[%.1f, %.1f]). Intente de nuevo.", x_min+walls, x_max-walls, ...
                        y_min+walls, y_max-walls);
                end
            else
                warning("Entrada inválida. Debe ingresar un vector [x y].");
            end
        end
        break;

    else
        fprintf("Opción inválida. Debe elegir 1, 2 o 3.\n");
    end
end


% Convertir a celdas
world2grid = @(p) [ round( (-p(1)+x_max)*100 ), round( (p(2)+y_max)*100 ) ]; % -> (row,col)

start_rc = world2grid(start_world); 
goal_rc  =  world2grid(goal_world);

%% -------------------------- Planificador D* (grid) --------------------------
disp("Iniciando el planificador D estrella");
% Implementación Dstar
dx = Dstar(map);
% Establecer meta
dx.plan(goal_rc);

% Consultar ruta desde el inicio 
path_rc=dx.query(start_rc, 'animate');

% Convertir ruta a coordenadas de mundo (m)
grid2world = @(rc) [ (+x_max*100-rc(1))/100, (-y_max*100+rc(2))/100 ]; % -> (x,y)
p = zeros(size(path_rc,1), 2);
for k=1:size(path_rc,1)
    p(k,:) = grid2world(path_rc(k,:)); % [x y]
end

% Suavizar ruta con spline
t_pts = linspace(0,1,size(p,1));
ppx = spline(t_pts, p(:,1)');
ppy = spline(t_pts, p(:,2)');

n_samples = max(200, round( norm(goal_world-start_world)/0.01 ));
ts = linspace(0,1,n_samples);
traj_x = ppval(ppx, ts);
traj_y = ppval(ppy, ts);

% Construir path en el formato p como columnas [x(mm), y(mm)] 
p_mm = [traj_x(:)*1000, traj_y(:)*1000]; % mm
input("Planificación lista. Presione enter para continuar, cerrar las figuras anteriores y " + ...
    "eliminar los datos de la consola.")
close all;
clc;

%% ---------------------- Configuración bloqueo rotación ----------------------
while true
    rotate_option = input("Desea que el robot gire:\n  1. Solo al llegar a la meta\n " + ...
        "2. Nunca\n Opción: ");
    if rotate_option == 1
        rotate_policy = 'only_at_goal';
        while true
            theta_final=input("¿Qué ángulo en grados (°) desea al final? (Entre 0 y 360)");
            if isnumeric(theta_final) && theta_final >= 0 && theta_final <= 360
                fprintf("Ángulo final establecido en %.2f°.\n",theta_final);
                theta_final=wrapToPi(deg2rad(theta_final));
                break;
            else
                disp("Valor inválido");
            end
        end

        break;
    elseif rotate_option == 2
        rotate_policy = 'never';
        break;
    else
        fprintf("Opción inválida, debe elegir 1, 2, 3 o 4.\n");
    end
end
block_theta=false;
fprintf("Política de rotación seleccionada: %s\n", rotate_policy);

% Estado inicial
state = [start_world(1); start_world(2); th]; % [x;y;theta]

% Control (pure pursuit + PI en velocidad cuerpo)
control = "PI"; 

% Pure pursuit param
lookahead_dist = 190/2; % mm

% --- Tolerancia para métricas de desempeño (distancia entre deseado y real)
tol_mm = lookahead_dist*2; % mm 

%% ----------------- PID en velocidad cuerpo (PI+FeedForward) -----------------
%kps
kp_phi = 0.8; kp_x = 1.6; kp_y = 1.6;
%kis
ki_phi = 0.15; ki_x = 0.15; ki_y = 0.15;
%Matrices de control
Kp = diag([kp_x, kp_y, kp_phi]);
Ki = diag([ki_x, ki_y, ki_phi]);
eP=0;
eO=0;


% Para visualización del robot
robot_w = 0.19; robot_h = 0.19; % m
base_rect = [-robot_h/2, -robot_w/2;
              robot_h/2, -robot_w/2;
              robot_h/2,  robot_w/2;
             -robot_h/2,  robot_w/2]';

%% - Figura del mapa - las trayectorias y la animación de la plataforma móvil -
disp("Iniciando figura de las trayectorias y la animación");
fig_map = figure('Name','Mapa y planificación','Color','w', 'Position',[2 2 638 638]);
subplot(1,2,1);
imagesc(linspace(x_min,x_max), linspace(y_min,y_max), fliplr(map)); axis xy; axis equal;
hold on; colormap(gray); title('Mapa (0=libre,1=ocupado)');
plot(start_world(1), start_world(2), 'go', 'MarkerFaceColor','g');
plot(goal_world(1), goal_world(2), 'rx', 'MarkerSize',10, 'LineWidth',2);
plot(p(:,1), p(:,2), 'b.', 'LineWidth',1.2); % ruta coarsed antes de spline
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

hTime = text(-map_w/2+0.05, map_h/2-0.15, '', ...
    'FontName','Times New Roman', 'FontSize', 8, 'FontWeight','bold');
hVelRuedas = text(-map_w/2+0.05, map_h/2-0.45, '', ...
    'FontName','Times New Roman', 'FontSize', 8);

%% ----------------------- Movimiento de la plataforma ------------------------
% Preparar interrupción por tecla (ENTER) para emergencia
disp("Preparado para iniciar el control, puede presionar enter sobre la figura para parado" + ...
    "de emergencia.");
input("Presione enter para iniciar en 2 segundos");
pause(2);
stop_flag = false;
listener = @(~,~) assignin('base','stop_flag', true);
set(gcf, 'KeyPressFcn', listener);
disp('Ventana activa: presiona ENTER para STOP de emergencia.');

robot_started=false;

% Inicializaciones para el bucle
idx_goal = 1;
n_pts = size(p_mm,1);  % ruta ya calculada 
Ierr = zeros(3,1);     % integrador
I_max = [1.0; 0.5; 0.5]; I_min = -I_max;  % límites integrador (ajustables)

% ---- INICIALIZACIÓN DE LOGS PARA REGISTRO Y PLOTS FINALES ----

max_iter = ceil(300/dt);
Xlog = zeros(6, max_iter);        % x, y, theta, idx, xg, yg  [m, m, rad, , m, m]
Errlog = zeros(1, max_iter);      % eP [mm]
CTE_log = zeros(1, max_iter);     % CTE absoluto [mm]
CTE_signed_log = zeros(1, max_iter); % CTE firmado [mm] (izq > 0, der < 0)
rpm_log = zeros(4, max_iter);     % rpm ruedas
time_log = zeros(1, max_iter);    % tiempo [s]

log_idx = 0;
% -------------------------------------------------------------

% Main loop (hardware-only)
try
    for k = 1:ceil(300/dt)  % límite por seguridad. 
        loop_t0 = tic;

        % --------- 1) Leer pose real del robot desde Robotat ----------
        robot_pos = robotat_get_pose(robotat, id, 'eulzyx'); 
        x_real = (robot_pos(1) - offset(1));   % m
        y_real = (robot_pos(2) - offset(2));   % m
        th_real = deg2rad(atan2d(sind(robot_pos(4)), cosd(robot_pos(4)))-offset(4)); % rad
        state = [x_real; y_real; th_real];

        % --------- 2) Seleccionar objetivo lookahead----------
        while idx_goal < n_pts
            pm_tmp = p_mm(idx_goal,:);       % [x_mm y_mm]
            pm_tmp_m = pm_tmp/1000;          % m
            dx = pm_tmp_m(1) - state(1);
            dy = pm_tmp_m(2) - state(2);
            if hypot(dx*1000, dy*1000) >= lookahead_dist
                break;
            end
            idx_goal = idx_goal + 1;
        end      

        % punto meta en metros
        pm = p_mm(idx_goal,:)/1000; xg = pm(1); yg = pm(2);

        % pose actual (determinada por Robotat)
        x = state(1); y = state(2); th = state(3);

        % errores
        e = [xg - x; yg - y];
        eP = norm(e)*1000; % mm

        switch rotate_policy
            case 'only_at_goal'
                block_theta = ~(idx_goal == n_pts && eP < 1.5*lookahead_dist);
            case 'never'
                block_theta = true;
            otherwise
                block_theta = true;
        end

        if block_theta
            thetag = 0;
        else
            thetag = theta_final;
        end

        eO = wrapToPi(thetag - th);

        if idx_goal >= n_pts && eP < lookahead_dist/2 && norm(eO)<deg2rad(5)
            % ruta consumida -> detener robot
            u_rpm = zeros(4,1);
            robot_set_velocities(u_rpm);    % enviar parada
            disp('Ruta consumida -> parada física.');
            % registrar estado final antes de salir
            log_idx = log_idx + 1;
            Xlog(:,log_idx) = [state; idx_goal; xg; yg];
            Errlog(1,log_idx) = 0;
            rpm_log(:,log_idx) = u_rpm;
            time_log(log_idx) = (k-1)*dt;
            break;
        end

        % ------------------ Control PI ------------------
        % Error en mundo [x; y; theta]
        err = [ e(1); e(2); eO ];
        
        % Integrador candidato
        Ierr_cand = Ierr + err * dt;
        Ierr_cand = max(min(Ierr_cand, I_max), I_min);

        % velocidad meta (estimada entre idx_goal y el siguiente punto)
        if idx_goal < n_pts
            pm_next = p_mm(idx_goal+1,:) / 1000;   % siguiente punto en metros
            v_des = (pm_next - pm) / dt;            % velocidad [m/s] por componente
            % velocidad angular deseada (rad/s)
            if block_theta
                w_des = 0.2 * eO;
            else
                % calculamos el ángulo entre los dos puntos y su cambio
                theta_current = thetag;
                theta_next    = atan2(pm_next(2)-pm(2), pm_next(1)-pm(1));
                w_des = wrapToPi(theta_next - theta_current) / dt;  % rad/s
            end
            %Escalar
            if norm(v_des) > v_max
                scale = v_max / norm(v_des);
                v_des = scale * v_des;   % escala ambos componentes proporcionalmente
            end
            if abs(w_des)>w_max
                scale=abs(w_max/w_des);
                w_des=scale*w_des;
            end
        else
            v_des = [0 0]; % último punto, velocidad cero
            w_des=0;
        end
        %FeedForward
        qd_dot_world = [ v_des(1); v_des(2); w_des ];

        % Ley FeedForward+PI
        qdot_world = qd_dot_world + Kp * err + Ki * Ierr_cand;

        % mundo -> cuerpo
        R_th = [cos(th), -sin(th); sin(th), cos(th)];
        v_body = R_th' * qdot_world(1:2);    % [v_bx; v_by]
        omega = qdot_world(3);
        Vb_cmd = [omega; v_body(1); v_body(2)];

        % --------- 3) Convertir a RPMs y saturar (anti-windup) ----------
        u_rad_s_cand = (1/r) * H0 * Vb_cmd;
        u_rpm_cand = u_rad_s_cand * 60 / (2*pi);

        if any(abs(u_rpm_cand) > rpm_max)
            scale = rpm_max / max(abs(u_rpm_cand));
            u_rpm = u_rpm_cand * scale;
            u_rad_s = u_rpm * 2*pi / 60;
            Vb_real = r * H0_pinv * u_rad_s;
            % NO actualizar integrador (anti-windup simple)
        else
            u_rpm = u_rpm_cand;
            u_rad_s = u_rpm * 2*pi / 60;
            Vb_real = Vb_cmd;
            % actualizar integrador porque la salida es válida
            Ierr = Ierr_cand;
        end

        % --------- 4) Enviar RPMs al robot físico (robot_set_velocities) ----------
        % robot_set_velocities espera un vector 
        try
            robot_start();
            robot_set_velocities(u_rpm);   % envío directo a hardware
            if robot_started==false
                robot_start();
                robot_started=true;
            end
        catch ME
            warning('robot_set_velocities fallo: %s', ME);
            % En caso de fallo, detener y salir seguro
            try
                robot_stop(); 
                robot_started=false;
            catch e
                disp(e);
            end
            break;
        end

        % --------- 5) Mostrar info y protección de parada ----------
        % Actualizar elementos visuales con la pose real
        R = [cos(th), -sin(th); sin(th), cos(th)];
        pos = R * base_rect + state(1:2);
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

        % Actualizar texto con RPM en tiempo real (formato claro)
        set(hTime, 'String', sprintf('Tiempo: %.2f s | eP=%.0f mm | eO=%.2f deg', ...
            k*dt, eP, rad2deg(eO)));
        set(hVelRuedas, 'String', sprintf(['v=%.3f m/s  w=%.3f rad/s\nRPM (1..4): ' ...
            '%6.0f  %6.0f  %6.0f  %6.0f'], ...
            v_display, w_display, u_rpm(1), u_rpm(2), u_rpm(3), u_rpm(4)));
        drawnow limitrate;

        % ------------------ Calcular CTE respecto a segmento objetivo ------------------
        % Usamos p_mm en mm (la trayectoria suavizada) y la posición real P en mm.
        P_mm = [state(1:2)']*1000;

        % seleccionamos segmento A-B: entre punto objetivo actual y el siguiente
        if idx_goal < n_pts
            A = p_mm(idx_goal, :);          % mm
            B = p_mm(min(idx_goal+1, n_pts), :); % mm
        else
            % si estamos en el último punto, usar el segmento anterior
            A = p_mm(max(1, idx_goal-1), :);
            B = p_mm(idx_goal, :);
        end

        % función inline: proyección y distancia (evitar dividir por cero)
        AB = B - A; AP = P_mm - A;
        denom = dot(AB, AB);
        if denom == 0
            t_proj = 0;
            Q = A;
        else
            t_proj = dot(AP, AB) / denom;
            % limitamos a segmento
            t_clamped = max(0, min(1, t_proj));
            Q = A + t_clamped * AB;
        end
        vecPQ = P_mm - Q;
        cte_abs_mm = norm(vecPQ); % magnitud (mm)
        % signed cte (producto cruz 2D / |AB|)
        normAB = norm(AB);
        if normAB < eps
            cte_signed_mm = 0;
        else
            % AP usado para signo (no la proyección)
            cte_signed_mm = (AB(1)*AP(2) - AB(2)*AP(1)) / normAB;
        end
        
        % ---- REGISTRAR DATOS EN LOGS ----
        log_idx = log_idx + 1;               % 
        Xlog(:,log_idx) = [state; idx_goal; xg; yg]; % [x;y;theta]
        Errlog(1,log_idx) = eP;              % mm
        rpm_log(:,log_idx) = u_rpm ;         % rpm
        time_log(log_idx) = (k-1)*dt ;       % s
        CTE_log(log_idx) = cte_abs_mm;
        CTE_signed_log(log_idx) = cte_signed_mm;

        % ---------------------------------

        % Emergency stop por tecla ENTER (variable stop_flag en workspace)
        if evalin('base','exist(''stop_flag'',''var'') && stop_flag')
            disp('EMERGENCY STOP: tecla presionada. Deteniendo robot...');
            robot_stop();
            robot_started=false;
            break;
        end

        % Mantener tasa de control ~ dt
        elapsed = toc(loop_t0);
        if elapsed < dt
            pause(dt - elapsed);
        end
    end % for loop

catch ME
    % En caso de error: parar hardware y reportar
    warning(ME.identifier,'Error en bucle físico: %s', ME.message);
    try 
        robot_emergency_stop();
        robot_started=false;
    catch e
        disp(e);
    end
end

% Salida limpia: detener y desconectar
try
    robot_stop();
    pause(0.05);
    robot_stop();
    robot_started=false;
catch, end

% Quitar listener de tecla
set(gcf, 'KeyPressFcn', '');
clear stop_flag;
disp('Ejecución finalizada. Robot detenido.');

% ---------------- Mostrar valores finales en la consola ----------------
if log_idx > 0

    final_pose = Xlog(1:3, log_idx); % [x;y;theta]
    final_rpms = rpm_log(:, log_idx)';
    
    fprintf('Final X = %.3f m \n', final_pose(1));
    fprintf('Final Y = %.3f m \n', final_pose(2));
    fprintf('Final Theta = %.2f deg \n', rad2deg(final_pose(3)));
    fprintf('Final RPMs = %6.0f  %6.0f  %6.0f  %6.0f \n', final_rpms)
        
end

%% ------------------------------- Desconectar --------------------------------
robot_disconnect();
robotat_disconnect(robotat);

%% ------------------- Preguntar si desea ver resumen final -------------------
while true
    show_summary = input("¿Desea ver el resumen final del movimiento? (Y/N): ", 's');
    show_summary = upper(strtrim(show_summary)); % asegurar mayúsculas y quitar espacios
    if show_summary == "Y"
        % ---------------- Mostrar valores finales en la figura del mapa ----------------
        if log_idx > 0
            final_pose = Xlog(1:3, log_idx); % [x;y;theta]
            final_rpms = rpm_log(:, log_idx)';
            fprintf('Final X = %.3f m \n', final_pose(1));
            fprintf('Final Y = %.3f m \n', final_pose(2));
            fprintf('Final Theta = %.2f deg \n', rad2deg(final_pose(3)));
            fprintf('Final RPMs = %6.0f  %6.0f  %6.0f  %6.0f \n', final_rpms)

            % --- Métricas de desempeño ---
            % Posición (Errlog está en mm)
            pos_err_mm = Errlog(1:log_idx); % mm

            % tomar sólo muestras donde eP >= lookahead_dist (estado "activo")
            active_idx = pos_err_mm >= lookahead_dist;

            RMSE_pos_mm = sqrt(mean((pos_err_mm(active_idx) - lookahead_dist).^2));
            MaxErr_pos_mm = max(abs(pos_err_mm));

            fprintf('\n--- Métricas globales ---\n');
            fprintf('RMSE posición = %.2f mm (%.3f m)\n', RMSE_pos_mm, RMSE_pos_mm/1000);
            fprintf('Error máximo posición = %.2f mm (%.3f m)\n', MaxErr_pos_mm, MaxErr_pos_mm/1000);
            %fprintf('RMSE CTE = %.2f mm (%.3f m)\n', RMSE_cte_mm, RMSE_cte_mm/1000);
            %fprintf('Error máximo CTE = %.2f mm (%.3f m)\n', MaxErr_cte_mm, MaxErr_cte_mm/1000);
        else
            disp("No hay datos de log para mostrar el resumen final.");
        end
        break;
    elseif show_summary == "N"
        disp("Resumen final omitido.");
        break;
    else
        fprintf("Opción inválida. Ingrese 'Y' o 'N'.\n");
    end
end

%% ------------------------------- Plots finales ------------------------------ 
if log_idx > 0 && show_summary == "Y"
    figure('Name','Resultados','Color','w', ...
       'DefaultAxesFontName','Times New Roman', ...
       'DefaultTextFontName','Times New Roman', ...
       'DefaultAxesFontSize',12, ...
       'DefaultTextFontSize',12, ...
       'Position',[2 2 638 638]);

    % 1) Trayectoria deseada vs real
    subplot(2,2,[1 3]);
    plot(traj_x, traj_y,'r--','LineWidth',1); hold on;
    plot(Xlog(1,1:log_idx), Xlog(2,1:log_idx),'b-','LineWidth',1.5);
    axis equal; grid on; xlabel('X [m]'); ylabel('Y [m]'); title('Trayectoria deseada vs real');
    xlim([x_min x_max]);
    ylim([y_min y_max]);
    legend('Deseada','Real');

    % 3) RPM plot
    subplot(2,2,[2 4]);
    plot(time_log(1:log_idx), rpm_log(:,1:log_idx)');
    xlabel('Tiempo [s]'); ylabel('RPM'); title('RPM ruedas'); grid on;

    sgtitle('Resumen simulación');

    % --- Preparar datos para el plot de errores ---
    t = time_log(1:log_idx);
    % --- Métricas de desempeño ---
    % Posición (Errlog está en mm)
    pos_err_mm = Errlog(1:log_idx); % mm
    
    % tomar sólo muestras donde eP >= lookahead_dist (estado "activo")
    active_idx = pos_err_mm >= lookahead_dist;

    RMSE_pos_mm = sqrt(mean((pos_err_mm(active_idx) - lookahead_dist).^2));
    

    eP_series = Errlog(1:log_idx);              % mm
    [max_eP_mm, idx_max] = max(eP_series);
    t_max = t(idx_max);

    CTE_series = CTE_log(1:log_idx);            % mm
    RMSE_cte_mm = sqrt(mean(CTE_series.^2));   % mm
    
    figure('Name','Resultados_2','Color','w', ...
       'DefaultAxesFontName','Times New Roman', ...
       'DefaultTextFontName','Times New Roman', ...
       'DefaultAxesFontSize',12, ...
       'DefaultTextFontSize',12, ...
       'Position',[640 2 638*2 638]);

    % 2) Plot: eP + lookahead + RMSEs+lookahead + punto máximo
    
    hold on;
    plot(t, eP_series, 'blue', 'LineWidth', 1.2); % eP en azul

    % lookahead (color [95 164 56]/255) con '-.'
    yline(lookahead_dist, '-.', 'LineWidth', 1.5, 'Color', [95,164,56]/255);

    % RMSE_pos + lookahead en rojo '--'
    yline(RMSE_pos_mm + lookahead_dist, '--', 'LineWidth', 1.5, 'Color', 'r');

    % RMSE_cte + lookahead en magenta '--'
    %yline(RMSE_cte_mm + lookahead_dist, '--', 'LineWidth', 1.5, 'Color', 'm');

    % punto del máximo de eP (círculo rojo)
    plot(t_max, max_eP_mm, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);

    xlabel('Tiempo [s]'); ylabel('eP [mm]'); title('Error posición (mm) con límites');
    grid on; box on;
    legend({'eP (mm)', 'Lookahead', 'RMSE', 'eP_{máx}'}, 'Location', 'best');

    
end

%% ------------------------- Guardar datos y figuras --------------------------
while true
    save_opt = upper(strtrim(input("¿Desea guardar los datos y las figuras? (Y/N): ", 's')));
    if save_opt == "Y"
        % timestamp para el nombre (hora)
        ts = datetime('now', 'Format','yyyy-MM-dd-HH_mm_ss');
        % carpetas solicitadas
        fig_dir = fullfile(pwd, 'Figuras');
        data_dir = fullfile(pwd, 'Datos');
        if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end
        if ~exist(data_dir, 'dir'), mkdir(data_dir); end

        % nombre base: Figura_traj_choice_rotate_policy_hora (con valores reales)
        base_name = sprintf('Figura_Mapa_%s_%s', rotate_policy, ts);

        % --- Guardar figura del mapa (fig_map) si existe ---
        if exist('fig_map','var') && isvalid(fig_map)
            try
                exportgraphics(fig_map, fullfile(fig_dir, [base_name '_map.png']));
            catch
                saveas(fig_map, fullfile(fig_dir, [base_name '_map.png']));
            end
            try
                savefig(fig_map, fullfile(fig_dir, [base_name '_map.fig']));
            catch
                % noop
            end
        end

        % --- Guardar figura "Resultados" si existe ---
        hres = findobj('Type','figure','-and','Name','Resultados');
        if ~isempty(hres)
            hres = hres(1); % tomar la primera si hay más
            try
                exportgraphics(hres, fullfile(fig_dir, [base_name '_resultados.png']));
            catch
                saveas(hres, fullfile(fig_dir, [base_name '_resultados.png']));
            end
            try
                savefig(hres, fullfile(fig_dir, [base_name '_resultados.fig']));
            catch
                % noop
            end
        end
        % --- Guardar figura "Resultados" si existe ---
        hres = findobj('Type','figure','-and','Name','Resultados_2');
        if ~isempty(hres)
            hres = hres(1); % tomar la primera si hay más
            try
                exportgraphics(hres, fullfile(fig_dir, [base_name '_resultados_2.png']));
            catch
                saveas(hres, fullfile(fig_dir, [base_name '_resultados_2.png']));
            end
            try
                savefig(hres, fullfile(fig_dir, [base_name '_resultados_2.fig']));
            catch
                % noop
            end
        end

        % --- Guardar datos (mat + CSVs) ---
        try
            if exist('log_idx','var') && log_idx>0
                Xlog_s    = Xlog(:,1:log_idx);
                Errlog_s  = Errlog(1:log_idx);
                rpm_log_s = rpm_log(:,1:log_idx);
                time_log_s= time_log(1:log_idx);
            else
                Xlog_s = []; Errlog_s = []; rpm_log_s = []; time_log_s = [];
            end
            datafile = fullfile(data_dir, sprintf('Datos_Mapa_%s_%s.mat', rotate_policy, ts));
            save(datafile);

            % CSVs para acceso rápido (si hay datos)
            if ~isempty(Xlog_s)
                writematrix(Xlog_s', fullfile(data_dir, sprintf('Xlog_Mapa_%s_%s.csv', ...
                    rotate_policy, ts)));
                writematrix(Errlog_s',  fullfile(data_dir, sprintf('Errlog_Mapa_%s_%s.csv', ...
                    rotate_policy, ts)));
                writematrix(rpm_log_s,  fullfile(data_dir, sprintf('rpm_log_Mapa_%s_%s.csv', ...
                    rotate_policy, ts)));
                writematrix(time_log_s', fullfile(data_dir, sprintf('time_log_Mapa_%s_%s.csv', ...
                    rotate_policy, ts)));
            end
        catch ME
            warning('Error guardando datos: %s', ME.message);
        end

        fprintf('Guardado completado.\n Figuras -> %s\n Datos -> %s\n', fig_dir, data_dir);
        break;

    elseif save_opt == "N"
        fprintf('Guardado omitido por el usuario.\n');
        break;

    else
        fprintf("Opción inválida. Ingrese 'Y' o 'N'.\n");
    end
end




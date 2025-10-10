% Físico : Control PI de plataforma omnidireccional (4 ruedas Mecanum) 
% + generación de mapa con obstáculos, planificación (D*) y trayectoria
% Animación del robot (patch + trail) y visualización del mapa y ruta

close all; clc;
%clear;
disp("----------Seguimiento de Trayectorias con Control PI Robot Omnidireccional----------");
%%
disp("Conectando al Robotat");

timeout = 6;      % tiempo máximo de espera en segundos
t0 = tic;
try 
    robotat_disconnect(robotat);
catch ME
    fprintf("%s\n", ME.message);

end

while true
    
    try
        % Intentar conectar
        robotat = robotat_connect();
        fprintf("\nConexión exitosa en %.2f s\n", toc(t0));
        break; % salir del while si se conecta
    catch ME
        % Si falla la conexión
        if toc(t0) > timeout
            error("No se pudo conectar al Robotat en %d segundos.\nError: %s", timeout, ME.message);
        end
        fprintf("."); % imprimir punto mientras intenta
        pause(0.5);   % espera antes de reintentar
    end
end

clear t0;

%% Conexión a la plataforma móvil
disp("Conectando con el Robot");

timeout = 6;      % tiempo máximo de espera en segundos
t0 = tic;

try 
    robot_disconnect();
catch ME
    fprintf("%s\n", ME.message);

end

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

clear t0;
%% Visualización inicial en el robotat
disp("Visualización del robot");
id=29; %ID del robot en el robotat
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


%% -------------------- Parámetros del mundo / mapa --------------------
map_w = 4.0;   % ancho en metros (X)
map_h = 5.0;   % alto en metros (Y)
% Origen del mundo: colocamos (0,0) en el centro del mapa para la simulaci?n
x_min = -map_w/2; x_max = map_w/2;
y_min = -map_h/2; y_max = map_h/2;
walls=0.1; %Offsets de las paredes para que el Robot no tope con ellas

%% ----- Definir obstáculos (centros en m) y visualizar mapa -----
disp("Definición de los obstáculos y visualización del mapa");

% Obstáculos iniciales
ids = [66 65];  % marcadores de obstáculos en Robotat

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
    'FontSize', 14, 'Interpreter', 'latex');
xlabel('$x$ (en cm)', 'FontSize', 14, 'Interpreter', 'latex');
ylabel('$y$ (en cm)', 'FontSize', 14, 'Interpreter', 'latex');

%legend();
axis on;

%% -------------------- Inicio y meta (en metros) --------------------
disp("Definiendo inicio (Robot) y meta:");

% Cargar offsets y obtener pose actual
load Offsets; 
robot_pos = robotat_get_pose(robotat, id, 'eulzyx');
offset = offsets_m;

xpos = (robot_pos(1) - offset(1)) * 1000; % mm
ypos = (robot_pos(2) - offset(2)) * 1000; % mm
yaw  = atan2d(sind(robot_pos(4) - offset(4)), cosd(robot_pos(4) - offset(4))); % °

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
            fprintf("Meta establecida en marcador %d: [%.3f, %.3f] m\n", goal_id, goal_world(1), goal_world(2));
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
                    warning("⚠️ Coordenada fuera de los límites del mapa (X:[%.1f, %.1f], " + ...
                        "Y:[%.1f, %.1f]). Intente de nuevo.", x_min+walls, x_max-walls, y_min+walls, y_max-walls);
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


%% -------------------- Planificador D* (grid) --------------------
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

% Suavizar ruta con spline (opcional)
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
%% -------------------- Parámetros fí­sicos del robot --------------------
w = ((56.8+81.8)/2)/1000;   % m
l = ((74.65+32.35)/2)/1000; % m
r = 48/1000;                % radio rueda [m]
% Lí­mites motores
rpm_max = 80;    % rpm

H0 = [ -l - w, 1, -1;
        l + w, 1,  1;
        l + w, 1, -1;
       -l - w, 1,  1 ];
H0_pinv = pinv(H0);

%% -------------------- Parámetros de la simulación --------------------
% Simulación
dt = 0.02;        % s
Tsim = 30;        % s
time = 0:dt:Tsim;

% ---------------- Configuración bloqueo rotación ----------------
while true
    rotate_option = input("Desea que el robot gire:\n  1. Solo al llegar a la meta\n  2. Tarde\n  3. Nunca\nOpción: ");
    if rotate_option == 1
        rotate_policy = 'only_at_goal';
        break;
    elseif rotate_option == 2
        rotate_policy = 'late';
        break;
    elseif rotate_option == 3
        rotate_policy = 'never';
        break;
    else
        fprintf("⚠️ Opción inválida, debe elegir 1, 2 o 3.\n");
    end
end

rotate_fraction = 0.7; % si policy 'late', permitir rotar cuando idx_goal >= rotate_fraction*n_pts
fprintf("Política de rotación seleccionada: %s\n", rotate_policy);

% Estado inicial
state = [start_world(1); start_world(2); 0]; % [x;y;theta]

% Control (pure pursuit + PI en velocidad cuerpo)
control = "PI"; 

% Pure pursuit param
lookahead_dist = 190; % mm

% PID en velocidad cuerpo (PI) - ganancias iniciales
%kps
kp_phi = 8.0*0; kp_x = 1.6; kp_y = 1.6/2;
%kis
ki_phi = 0.5*0; ki_x = 0.15; ki_y = 0.15;
%Matrices de control
Kp = diag([kp_x, kp_y, kp_phi])/10;
Ki = diag([ki_x, ki_y, ki_phi])/10;


% Para visualización del robot
robot_w = 0.19; robot_h = 0.19; % m
base_rect = [-robot_h/2, -robot_w/2;
              robot_h/2, -robot_w/2;
              robot_h/2,  robot_w/2;
             -robot_h/2,  robot_w/2]';

%% Figura del mapa-las trayectorias y la animación de la plataforma móvil
disp("Iniciando figura de las trayectorias y la animación");
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

hTime = text(-map_w/2+0.05, map_h/2-0.15, '', 'FontSize', 10, 'FontWeight','bold');
hVelRuedas = text(map_w/2-0.6, map_h/2-0.3, '', 'FontSize',9, 'FontName','Courier New');

%% Movimiento de la plataforma
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
Xlog = zeros(3, max_iter);        % x, y, theta [m, m, rad]
Errlog = zeros(1, max_iter);      % eP [mm]
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
        th_real = deg2rad(robot_pos(4) - offset(4)); % rad
        state = [x_real; y_real; wrapToPi(th_real)];

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

        if idx_goal >= n_pts
            % ruta consumida -> detener robot
            u_rpm = zeros(4,1);
            robot_set_velocities(u_rpm);    % enviar parada
            disp('Ruta consumida -> parada física.');
            % registrar estado final antes de salir
            log_idx = log_idx + 1;
            Xlog(:,log_idx) = state;
            Errlog(1,log_idx) = 0;
            rpm_log(:,log_idx) = u_rpm;
            time_log(log_idx) = (k-1)*dt;
            break;
        end

        % punto meta en metros
        pm = p_mm(idx_goal,:)/1000; xg = pm(1); yg = pm(2);

        

        % pose actual (determinada por Robotat)
        x = state(1); y = state(2); th = state(3);

        % errores
        e = [xg - x; yg - y];
        eP = norm(e)*1000; % mm
        thetag = atan2(e(2), e(1));
        eO = wrapToPi(thetag - th);

        % ------------------ Control PI (hardware) ------------------
        % Error en mundo [x; y; theta]
        err = [ e(1); e(2); eO ];

        switch rotate_policy
            case 'only_at_goal'
                block_theta = ~(idx_goal == n_pts && eP < lookahead_dist);
            case 'late'
                block_theta = idx_goal < max(1, round(rotate_fraction * n_pts));
            case 'never'
                block_theta = true;
        end

        % Bloqueo del error de theta
        err_tmp = err;
        if block_theta
            err_tmp(3) = 0;
        end

        % Integrador candidato
        Ierr_cand = Ierr + err_tmp * dt;
        Ierr_cand = max(min(Ierr_cand, I_max), I_min);
        if block_theta
            Ierr_cand(3) = Ierr(3);
        end

        % velocidad meta (estimada entre idx_goal y el siguiente punto)
        if idx_goal < n_pts
            pm_next = p_mm(idx_goal+1,:) / 1000;   % siguiente punto en metros
            v_des = (pm_next - pm) / dt;            % velocidad [m/s] por componente
            % velocidad angular deseada (rad/s)
            % calculamos el ángulo entre los dos puntos y su cambio
            theta_current = atan2(pm(2)-state(2), pm(1)-state(1));
            theta_next    = atan2(pm_next(2)-state(2), pm_next(1)-state(1));
            w_des = wrapToPi(theta_next - theta_current) / dt;  % rad/s
        else
            v_des = [0 0]; % último punto, velocidad cero
            w_des=0;
        end
        %FeedForward
        qd_dot_world = [ v_des(1); v_des(2); w_des ];

        % Valor deseado sin FeedForward sino como valor dependiende del
        % error
        %v_des = 2.0 * eP/1000;   % m/s
        %w_des = 0.2 * eO;        % rad/s
        %qd_dot_world = [ v_des*cos(thetag); v_des*sin(thetag); w_des ];

        % Ley FeedForward+PI
        qdot_world = qd_dot_world + Kp * err + Ki * Ierr_cand;
        if block_theta
            qdot_world(3) = 0;
        end

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
        % robot_set_velocities espera un vector (ajusta si tu API pide fila o columna)
        try
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

        v_display = norm([Vb_real(2), Vb_real(3)]);
        w_display = Vb_real(1);

        % Actualizar texto con RPM en tiempo real (formato claro)
        set(hTime, 'String', sprintf('Tiempo: %.2f s | eP=%.0f mm | eO=%.2f deg', k*dt, eP, rad2deg(eO)));
        set(hVelRuedas, 'String', sprintf('v=%.3f m/s  w=%.3f rad/s\nRPM (1..4): %6.0f  %6.0f  %6.0f  %6.0f', v_display, w_display, u_rpm(1), u_rpm(2), u_rpm(3), u_rpm(4)));
        drawnow limitrate;

        % ---- REGISTRAR DATOS EN LOGS ----
        log_idx = log_idx + 1;
        Xlog(:,log_idx) = state;               % [x;y;theta]
        Errlog(1,log_idx) = eP;               % mm
        rpm_log(:,log_idx) = u_rpm;           % rpm
        time_log(log_idx) = (k-1)*dt;         % s
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
        robot_stop(); 
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

% ---------------- Mostrar valores finales en la figura del mapa ----------------
if log_idx > 0
    final_pose = Xlog(:, log_idx); % [x;y;theta]
    final_rpms = rpm_log(:, log_idx)';
    % Mostrar texto en la subplot de animación (subpl 1,2)
    figure(fig_map);
    subplot(1,2,2);
    txt = {
        sprintf('Final X = %.3f m', final_pose(1));
        sprintf('Final Y = %.3f m', final_pose(2));
        sprintf('Final Theta = %.2f deg', rad2deg(final_pose(3)));
        sprintf('Final RPMs = %6.0f  %6.0f  %6.0f  %6.0f', final_rpms)
        };
    % Posición del texto (ajusta según gustes)
    text(-map_w/2+0.05, -map_h/2+0.15, txt, 'FontSize', 10, 'BackgroundColor','w','EdgeColor','k');
end

%% -------------------- Preguntar si desea ver resumen final --------------------
while true
    show_summary = input("¿Desea ver el resumen final del movimiento? (Y/N): ", 's');
    show_summary = upper(strtrim(show_summary)); % asegurar mayúsculas y quitar espacios
    if show_summary == "Y"
        % ---------------- Mostrar valores finales en la figura del mapa ----------------
        if log_idx > 0
            final_pose = Xlog(:, log_idx); % [x;y;theta]
            final_rpms = rpm_log(:, log_idx)';

            figure(fig_map);
            subplot(1,2,2);
            txt = {
                sprintf('Final X = %.3f m', final_pose(1));
                sprintf('Final Y = %.3f m', final_pose(2));
                sprintf('Final Theta = %.2f deg', rad2deg(final_pose(3)));
                sprintf('Final RPMs = %6.0f  %6.0f  %6.0f  %6.0f', final_rpms)
            };
            text(-map_w/2+0.05, -map_h/2+0.15, txt, 'FontSize', 10, 'BackgroundColor','w','EdgeColor','k');
        else
            disp("No hay datos de log para mostrar el resumen final.");
        end
        break;
    elseif show_summary == "N"
        disp("Resumen final omitido.");
        break;
    else
        fprintf("⚠️ Opción inválida. Ingrese 'Y' o 'N'.\n");
    end
end

%% -------------------- Plots finales --------------------
if log_idx > 0 && show_summary == "Y"
    figure('Name','Resultados','Color','w');
    subplot(2,2,1);
    plot(traj_x, traj_y,'r--','LineWidth',1); hold on;
    plot(Xlog(1,1:log_idx), Xlog(2,1:log_idx),'b-','LineWidth',1.5);
    axis equal; grid on; xlabel('X [m]'); ylabel('Y [m]'); title('Trayectoria deseada vs real');
    legend('Deseada','Real');

    subplot(2,2,2);
    plot(time_log(1:log_idx), Errlog(1:log_idx)); xlabel('Tiempo [s]'); ylabel('eP [mm]'); title('Error posición (mm)'); grid on;

    subplot(2,2,[3 4]);
    plot(time_log(1:log_idx), rpm_log(:,1:log_idx)'); xlabel('Tiempo [s]'); ylabel('RPM'); title('RPM ruedas'); grid on;

    sgtitle('Resumen simulación');
end


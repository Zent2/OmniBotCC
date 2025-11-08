% =========================================================================
% Seguimiento en tiempo real de marcador - Plataforma Omnidireccional
% (OmniBotCC)
% =========================================================================
% Propósito
%   Script para seguir en tiempo real la posición de un marcador (marker)
%   dentrol del Robotat y controlar la plataforma omnidireccional OmniBotCC
%   mediante un controlador PI+FeedForward sobre velocidades del cuerpo.
%
% Flujo principal
%   1) Conexión a Robotat y a la plataforma móvil (robot_connect / robotat_connect).
%   2) Lectura de offsets y pose inicial del robot.
%   3) Selección de un marcador objetivo (ID) y lectura de su pose en cada ciclo.
%   4) Control Pure-Pursuit simplificado + PI en velocidades (feedforward basado
%      en el error actual hacia el marcador).
%   5) Envío de RPMs a las ruedas (robot_set_velocities) y animación en tiempo real.
%   6) Logging de poses, errores y RPMs; parada de emergencia con ENTER.
%
% Uso rápido
%   - Ajustar parámetros en los bloques "Parámetros del mundo / mapa" y
%     "Parámetros de la simulación" (dt, Tsim, Kp, Ki, rpm_max, lookahead_dist).
%   - Ejecutar script, proporcionar ID del marcador cuando se solicite.
%   - Presionar ENTER sobre la figura para EMERGENCY STOP.
%
% Entradas / salidas (principales)
%   Entradas:
%     - id (ID propio del robot en Robotat)
%     - id_goal (ID del marcador objetivo a seguir)
%     - offsets (archivo Offsets.mat con offsets_m / offsets_mm)
%     - parámetros de control y simulación (Kp, Ki, rpm_max, lookahead_dist)
%   Salidas / logs:
%     - Xlog    : poses registradas [x; y; theta] (m, m, rad)
%     - Errlog  : error de posición (mm)
%     - rpm_log : RPM enviados/registrados por rueda
%     - time_log: tiempos por muestra
%
% Convenciones y unidades
%   - Posiciones en el mundo: metros (m). Internamente algunos vectores usan mm
%     para fines de visualización/log.
%   - Ángulos: yaw/lecturas de Robotat pueden venir en grados; el script convierte
%     a radianes (deg2rad) para cálculos trigonométricos.
%   - Theta = 0 se considera alineado con el eje X del Robotat (mapa).
%
% Seguridad y notas de ejecución
%   - El script realiza llamadas a funciones de hardware. Verifique conectividad
%     antes de ejecutarlo.
%   - Existe manejo de errores en envíos a robot_set_velocities y listener para
%     parada de emergencia (ENTER).
%   - Si el marcador no es visible en alguna iteración, el robot detiene el avance
%     (se envían ceros a las ruedas) y continúa intentando en la siguiente iteración.
%
% Consejos y depuración
%   - Ajuste Kp/Ki y lookahead_dist para mejorar comportamiento frente a ruido.
%   - Si la orientación no coincide con lo esperado, revise offsets y la
%     conversión de ángulos (deg<->rad).
%   - Para ver la orientación del robot en la animación se dibuja una flecha
%     (heading) sobre el rectángulo base del robot.
%
% Autor / versión
%   - Autor : Christian Campos
%   - Versión : 1.0 
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
disp("----------Seguimiento de un marcador en tiempo real con Control " + ...
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
pause(1);

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
        
    end
end
fprintf("\nADVERTENCIA: La conexión es UDP por lo que no está confirmada al 100% la " + ...
    "conexión, verifique que el Robot esté encendido \n");
pause(1);
clear t0;

%% ------------------ Visualización inicial en el robotat ---------------------
id=29; %ID del robot en el robotat
fprintf("\nVisualización del robot con ID: %d \n", id);

%Comprobar si se quiere o no visualizar el Robot en el Robotat.
while true
    visual = upper(input("¿Desea visualizar la posición del marcador en el Robotat? (Y/N): ", ...
        "s"));
    
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
%En este caso se usan solo para las gráficas
map_w = 4.0;   % ancho en metros (X)
map_h = 5.0;   % alto en metros (Y)
% Origen del mundo: colocamos (0,0) en el centro del mapa para la simulaci?n
x_min = -map_w/2; x_max = map_w/2;
y_min = -map_h/2; y_max = map_h/2;
%% ------------------------ Parámetros del mundo/mapa -------------------------
% Solo se usa para visualizar el mapa
% Obstáculos iniciales
obstacles=[];
map=genmap(obstacles);
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
%% ----------------- PID en velocidad cuerpo (PI+FeedForward) -----------------
%kps
kp_phi = 0.8; kp_x = 1.6; kp_y = 1.6;
%kis
ki_phi = 0.15; ki_x = 0.15; ki_y = 0.15;
%Matrices de control
Kp = diag([kp_x, kp_y, kp_phi]);
Ki = diag([ki_x, ki_y, ki_phi]);
lookahead_dist = 190/2; % mm
eP=lookahead_dist/2+1;
eO=deg2rad(45);

% Para visualización del robot
robot_w = 0.19; robot_h = 0.19; % m
base_rect = [-robot_h/2, -robot_w/2;
              robot_h/2, -robot_w/2;
              robot_h/2,  robot_w/2;
             -robot_h/2,  robot_w/2]';

%% ------------------ Seguimiento de marcador en tiempo real ------------------
disp("Se utilizará seguimiento del marcador objetivo en tiempo real.");
% Solicitar ID del marcador objetivo (si lo seleccionaste antes con menú, puedes
% reutilizar esa variable; aquí se pide explícitamente para evitar ambigüedades).
id_goal = input("Ingrese el ID del marcador objetivo a seguir (por ejemplo 66): ");

% No se realiza planificación global: la referencia será la pose del marcador
% leída en cada iteración del bucle principal.
disp("Referencia: pose del marcador. Se seguirá su posición en tiempo real.");
input("Preparado para iniciar seguimiento. Presione enter para continuar.");
clc;

%% ---------------------------- Inicio y marcador -----------------------------
disp("Definiendo inicio (Robot):");

% Cargar offsets y obtener pose actual
load Offsets; 
robot_pos = robotat_get_pose(robotat, id, 'eulzyx');
offset = offsets_m;

xpos = (robot_pos(1) - offset(1)) * 1000; % mm
ypos = (robot_pos(2) - offset(2)) * 1000; % mm
yaw  = atan2d(sind(robot_pos(4)), cosd(robot_pos(4)))- offset(4); % °
th=deg2rad(yaw);
robot_pos = [xpos, ypos, offset(3), yaw];
fprintf("Posición Actual del Robot:\n X = %.2f mm\n Y = %.2f mm\n Z = %.2f mm\n Yaw = %.2f °\n", ...
        xpos*1000, ypos*1000, offset(3)*1000, yaw);

goal_pose = robotat_get_pose(robotat, id_goal, 'eulxyz'); % en metros

fprintf("Posición Actual del Marcador:\n X = %.2f mm\n Y = %.2f mm\n Z = %.2f mm\n Yaw = %.2f °\n", ...
        goal_pose(1)*1000, goal_pose(2)*1000, goal_pose(3)*1000, goal_pose(4));

%% ---------------------- Configuración bloqueo rotación ----------------------
while true
    rotate_option = input("Desea que el robot gire:\n  1. Solo al llegar a la meta\n  2. Siempre \n  3. Nunca\nOpción: ");
    if rotate_option == 1
        rotate_policy = 'only_at_goal';
        break;
    elseif rotate_option == 2
        rotate_policy = 'always';
        break;
    elseif rotate_option == 3
        rotate_policy = 'never';
        break;
    else
        fprintf("⚠️ Opción inválida, debe elegir 1, 2 o 3.\n");
    end
end

fprintf("Política de rotación seleccionada: %s\n", rotate_policy);

% Estado inicial
state = [robot_pos(1); robot_pos(2); th]; % [x;y;theta]

%% - Figura del mapa-las trayectorias y la animación de la plataforma móvil -
disp("Iniciando figura de las trayectorias y la animación");
fig_map = figure('Name','Mapa y planificación','Color','w', 'Position',[2 2 638 638]);
subplot(1,2,1);
imagesc(linspace(x_min,x_max), linspace(y_min,y_max), map); axis xy; axis equal;
hold on; colormap(gray); title('Mapa (0=libre,1=ocupado)');
plot(robot_pos(1), robot_pos(2), 'go', 'MarkerFaceColor','g');
plot(goal_pose(1), goal_pose(2), 'rx', 'MarkerSize',10, 'LineWidth',2);
%plot(p(:,1), p(:,2), 'b-', 'LineWidth',1.2); % ruta coarsed antes de spline
plot(goal_pose(1), goal_pose(2), 'r--', 'LineWidth',1.0); % ruta suavizada
legend("Inicio", "Meta");
xlabel('X [m]'); ylabel('Y [m]');

subplot(1,2,2);
axis equal; axis([-map_w/2 map_w/2 -map_h/2 map_h/2]); grid on; hold on;
xlabel('X [m]'); ylabel('Y [m]'); title('Animación Robot');

hPatch = patch('XData',[], 'YData',[], 'FaceColor',[.6 .8 .9], 'EdgeColor','k');
hTrail = animatedline('Color','r','LineWidth',1.5);
hMeta = plot(goal_pose(1), goal_pose(2),'*r');
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

robot_started = false;

% Inicializaciones para el bucle
Ierr = zeros(3,1);     % integrador
I_max = [1.0; 0.5; 0.5]; I_min = -I_max;  % límites integrador 

% ---- INICIALIZACIÓN DE LOGS PARA REGISTRO Y PLOTS FINALES ----

max_iter = ceil(90/dt);
Xlog = zeros(3, max_iter);        % x, y, theta [m, m, rad]
Errlog = zeros(1, max_iter);      % eP [mm]
rpm_log = zeros(4, max_iter);     % rpm ruedas
time_log = zeros(1, max_iter);    % tiempo [s]
log_idx = 0;
% -------------------------------------------------------------

% Main loop: seguir el marcador en tiempo real
try
    for k = 1:max_iter  % límite por seguridad. 
        loop_t0 = tic;

        
        % --------- 1) Leer pose real del robot desde Robotat ----------
        robot_pos = robotat_get_pose(robotat, id, 'eulzyx'); 
        x_real = (robot_pos(1) - offset(1));   % m
        y_real = (robot_pos(2) - offset(2));   % m
        th_real = deg2rad(atan2d(sind(robot_pos(4)), cosd(robot_pos(4)))-offset(4)); % rad
        state = [x_real; y_real; th_real];

        % --------- 2) Leer pose objetivo (marcador) en tiempo real ----------
        goal_pose = robotat_get_pose(robotat, id_goal, 'eulxyz'); % en metros
        if isempty(goal_pose)
            % Si no se obtiene la pose, no avanzar; enviar 0s y continuar
            warning("No se pudo leer la pose del marcador %d en esta iteración.", id_goal);
            u_rpm = zeros(4,1);
            try
                robot_set_velocities(u_rpm);
            catch

            end
            % registrar y continuar
            log_idx = log_idx + 1;
            Xlog(:,log_idx) = state;
            Errlog(1,log_idx) = NaN;
            rpm_log(:,log_idx) = u_rpm;
            time_log(log_idx) = (k-1)*dt;
            continue;
        end

        % Referencia (metros)
        xg = goal_pose(1);
        yg = goal_pose(2);

        % pose actual (determinada por Robotat)
        x = state(1); y = state(2); th = state(3);

        % errores
        e = [xg - x; yg - y];
        eP = norm(e)*1000; % mm     

        % Determinar bloqueo de theta según la política seleccionada
        switch rotate_policy
            case 'only_at_goal'
                block_theta = ~(eP < 1.5*lookahead_dist);   % permitir rotar solo al estar cerca
            case 'never'
                block_theta = true;  % nunca permitir control de theta
            otherwise
                block_theta = false;
        end

        if block_theta
            thetag = 0;
            
        else
            thetag = atan2(e(2), e(1));
            
        end
        eO = wrapToPi(thetag - th);

        % --- Condición de parada: si estamos dentro del lookahead_dist -> parar
        if eP < lookahead_dist/2 && norm(eO)<deg2rad(5)
            % Envío parada y registro final
            u_rpm = zeros(4,1);
            try
                robot_set_velocities(u_rpm);
            catch
            end
            disp('Marcador alcanzado dentro del lookahead_dist -> parada física.');
            % registrar estado final antes de salir
            log_idx = log_idx + 1;
            Xlog(:,log_idx) = state;
            Errlog(1,log_idx) = eP;
            rpm_log(:,log_idx) = u_rpm;
            time_log(log_idx) = (k-1)*dt;
            break;
        end
        
        % ------------------ Control PI ------------------
        % Error en mundo [x; y; theta] (en unidades m, m, rad)
        err = [ e(1); e(2); eO ];

        % Integrador candidato con saturación
        Ierr_cand = Ierr + err * dt;
        Ierr_cand = max(min(Ierr_cand, I_max), I_min);

        % Como no hay trayectoria, el feedforward es simple basado en el error
        v_des = 0.25 * eP/1000;   % m/s (proporcional a la distancia)
        w_des = 0.2 * eO;        % rad/s (proporcional al error angular)
        qd_dot_world = [ v_des*cos(th_real); v_des*sin(th_real); w_des ];

        % Ley FeedForward + PI
        qdot_world = qd_dot_world + Kp * err + Ki * Ierr_cand;
        % convertir mundo -> cuerpo
        R_th = [cos(th), -sin(th); sin(th), cos(th)];
        v_body = R_th' * qdot_world(1:2);
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
        try
            robot_set_velocities(u_rpm);   % envío directo a hardware
            if robot_started==false
                robot_start();
                robot_started=true;
            end
        catch ME
            warning('robot_set_velocities fallo: %s', ME.message);
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

        % Actualizar texto con RPM en tiempo real
        set(hTime, 'String', sprintf('Tiempo: %.2f s | eP=%.0f mm | eO=%.2f deg', ...
            k*dt, eP, rad2deg(eO)));
        set(hVelRuedas, 'String', sprintf(['v=%.3f m/s  w=%.3f rad/s\nRPM (1..4): ' ...
            '%6.0f  %6.0f  %6.0f  %6.0f'], ...
            v_display, w_display, u_rpm(1), u_rpm(2), u_rpm(3), u_rpm(4)));
        drawnow limitrate;

        % ---- REGISTRAR DATOS EN LOGS ----
        log_idx = log_idx + 1;
        Xlog(:,log_idx) = state;               % [x;y;theta]
        Errlog(1,log_idx) = eP;               % mm
        rpm_log(:,log_idx) = u_rpm;           % rpm
        time_log(log_idx) = (k-1)*dt;         % s
        % ---------------------------------

        % Emergency stop por tecla ENTER
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
    % En caso de error: STOP
    warning(ME.identifier,'Error en bucle físico: %s', ME.message);
    try 
        robot_emergency_stop();
        robot_started=false;
    catch e
        disp(e);
    end
end
%% ------------------------------- Desconectar --------------------------------
robot_disconnect();
robotat_disconnect(robotat);


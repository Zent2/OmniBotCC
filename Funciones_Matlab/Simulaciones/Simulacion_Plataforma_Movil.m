%% Simulación interactiva de plataforma móvil omnidireccional con límite de velocidad de ruedas
close all; clc;

% ==== Parámetros físicos del robot ====
w = ((56.8+81.8)/2)/1000;   % Distancia desde el centro al eje longitudinal [m]
l = ((74.65+32.35)/2)/1000; % Distancia desde el centro al eje lateral [m]
r = 48/1000;                % Radio de la rueda [m]

% Matriz de transformación H0 y pseudoinversa
H0 = [ -l - w, 1, -1;
        l + w, 1,  1;
        l + w, 1, -1;
       -l - w, 1,  1 ];
H0_pinv = pinv(H0);

% ==== Velocidades iniciales ====
v_bx = 0;       % m/s
v_by = 0;       % m/s
omega_bz = 0;   % rad/s
rpm_max = 400;  % límite de rpm

% ==== Parámetros de simulación ====
dt = 0.05;          % paso de integración [s]
max_time = 120;     % duración máxima [s]
state = [0; 0; 0];  % Estado [x; y; theta]
t = 0;

% ==== Crear figura y controles ====
figure('Color','w','KeyPressFcn',@keyHandler, "WindowState","maximized");
axis equal;
axis([-3 3 -2.5 2.5]);
rectangle('Position', [-2.5, -2, 5, 4]); % área Robotat
grid on; hold on;
xlabel('X [m]'); ylabel('Y [m]');
title('Animación Robot Omnidireccional');

% Robot
robot_w = 190/1000;
robot_h = 190/1000;
base_rect = [-robot_h/2, -robot_w/2;
              robot_h/2, -robot_w/2;
              robot_h/2,  robot_w/2;
             -robot_h/2,  robot_w/2]'; 

hPatch = patch('XData',[], 'YData',[], ...
               'FaceColor',[.6 .8 .9], 'EdgeColor','k');
hTrail = animatedline('Color','r','LineWidth',1.5);

% ==== Textos en pantalla ====
hTime = text(-2.4, 2.2, '', 'FontSize', 10, 'FontWeight','bold');
hVelRuedas = text(+2.8, 2, '', 'FontSize', 9, 'FontName','Courier New');
hVelRobot  = text(+2.8, 1.4, '', 'FontSize', 9, 'FontName','Courier New');
hPosRobot = text(+2.8, 0.8, '', 'FontSize', 9, 'FontName','Courier New');

% Variable de control
running = true;

%% ==== Bucle de simulación ====
while running && t < max_time
    % Calcular velocidades de las ruedas (rad/s y rpm)
    Vb_actual = [omega_bz; v_bx; v_by];
    u_rad_s = (1/r) * H0 * Vb_actual;
    u_rpm = u_rad_s * 60 / (2*pi);

    % Limitar rpm a ±rpm_max
    if any(abs(u_rpm) > rpm_max)
        escala = rpm_max / max(abs(u_rpm));
        u_rpm = u_rpm * escala;
        Vb_actual = (r * H0_pinv * (u_rpm * 2*pi / 60));
        omega_bz = Vb_actual(1);
        v_bx = Vb_actual(2);
        v_by = Vb_actual(3);
    end

    % Cinemática global
    theta = state(3);
    vx = v_bx*cos(theta) - v_by*sin(theta);
    vy = v_bx*sin(theta) + v_by*cos(theta);

    % Integración Euler
    state(1) = state(1) + vx*dt;
    state(2) = state(2) + vy*dt;
    state(3) = state(3) + omega_bz*dt;

    % Actualizar dibujo del robot
    R = [cos(state(3)), -sin(state(3));
         sin(state(3)),  cos(state(3))];
    pos = R * base_rect + state(1:2);
    set(hPatch, 'XData', pos(1,:), 'YData', pos(2,:));
    addpoints(hTrail, state(1), state(2));

    % Actualizar tiempo
    t = t + dt;
    set(hTime, 'String', sprintf('Tiempo: %.2f s', t));

    % Mostrar velocidades ruedas
    set(hVelRuedas, 'String', sprintf(['Ruedas [rpm] (max %d):\n',...
        '1: %+6.2f   2: %+6.2f\n',...
        '3: %+6.2f   4: %+6.2f'], ...
        rpm_max, u_rpm(1), u_rpm(2), u_rpm(3), u_rpm(4)));

    % Mostrar posiciones del robot
    set(hPosRobot, 'String', sprintf(['x = %+5.3f m\n',...
        'y = %+5.3f m\n',...
        'yaw = %+5.3f rad'], state(1), state(2), state(3)));
    
    % Mostrar velocidades del robot
    set(hVelRobot, 'String', sprintf(['Robot:\n',...
        'v_x = %+5.3f m/s\n',...
        'v_y = %+5.3f m/s\n',...
        'ω_z = %+5.3f rad/s'], v_bx, v_by, omega_bz));

    drawnow;
    pause(dt);
end

disp('Simulación finalizada');

%% ==== Función para manejar teclas ====
function keyHandler(~, event)
    persistent vbx vby obz
    if isempty(vbx)
        vbx = 0; vby = 0; obz = 0;
    end

    switch event.Key
        case 'rightarrow',    vbx = vbx + 0.05; % aumentar X
        case 'leftarrow',  vbx = vbx - 0.05; % disminuir X
        case 'downarrow', vby = vby - 0.05; % mover Y+
        case 'uparrow',  vby = vby + 0.05; % mover Y-
        case 'a',          obz = obz + 0.05; % giro izq
        case 'd',          obz = obz - 0.05; % giro der
        case 'r',          vbx = 0; vby = 0; obz = 0; % reset
        case 'escape',     assignin('base','running',false); % salir
    end
    assignin('base','v_bx',vbx);
    assignin('base','v_by',vby);
    assignin('base','omega_bz',obz);
end

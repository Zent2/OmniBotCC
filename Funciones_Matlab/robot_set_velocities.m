function robot_set_velocities(rpm_vector)
    % Configura RPM y dirección (CW/CCW) para los 4 motores vía WiFi
    %
    % Parámetros:
    %   rpm_vector  : vector de 4 elementos con las velocidades en RPM
    %                 (signo positivo = CW, negativo = CCW)
    %
    % Ejemplo:
    %   robot_connect();
    %   robot_set_velocities([100, -120, 0, 80])

    global u esp32_ip esp32_port

    if isempty(u)
        error('No hay conexión activa. Llama a robot_connect() primero.');
    end

    if numel(rpm_vector) ~= 4
        error('Se requiere un vector de 4 elementos para los 4 motores.');
    end

    for i = 1:4
        rpm = abs(rpm_vector(i));
        dir = 'CW';
        if rpm_vector(i) < 0
            dir = 'CCW';
        end

        % Limitar RPM al máximo permitido por el ESP32
        rpm = min(rpm, 400);

        % Enviar dirección
        comando_dir = sprintf('m%d_dir=%s', i-1, dir);
        robot_send_WiFi(comando_dir);
        %pause(0.05);  % Pausa para evitar saturación

        % Enviar RPM
        comando_rpm = sprintf('m%d_rpm=%d', i-1, rpm);
        robot_send_WiFi(comando_rpm);
        %pause(0.05);
    end

    fprintf('Velocidades configuradas para los 4 motores vía WiFi.\n');
end

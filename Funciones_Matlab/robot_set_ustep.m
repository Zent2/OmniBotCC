function robot_set_ustep(ustep)
    % Configura la resolución de micropasos (8 o 32) para los 4 motores vía WiFi
    %
    % Parámetros:
    %   ustep : número de micropasos (solo se permite 8 o 32)
    %
    % Ejemplo:
    %   robot_connect();
    %   robot_set_ustep(32);

    global u esp32_ip esp32_port

    if isempty(u)
        error('No hay conexión activa. Llama a robot_connect() primero.');
    end

    if ustep ~= 8 && ustep ~= 32
        error('El valor de micropasos debe ser 8 o 32.');
    end

    comando_ustep = sprintf('step=%d', ustep);
    robot_send_WiFi(comando_ustep);

    fprintf('Micropasos configurados a %d para los 4 motores vía WiFi.\n', ustep);
end

function robot_send_WiFi(cmd)
    % Envía un comando UDP al ESP32

    global u esp32_ip esp32_port

    if isempty(u)
        disp('ERROR: No hay conexión activa. Llama a robot_connect() primero.');
        return;
    end

    if isstring(cmd)
        cmd = char(cmd);  % Asegura que sea tipo char
    end

    write(u, uint8(cmd), "uint8", esp32_ip, esp32_port);
end

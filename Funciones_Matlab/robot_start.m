function robot_start()
    global u esp32_ip esp32_port
    
    if isempty(u)
        error('No hay conexión activa. Llama a robot_connect() primero.');
    end
    % Inicia todos los motores
    robot_send_WiFi('start');
    fprintf('Robot iniciado y motores habilitados\n');
end

function robot_emergency_stop()
    % Detiene el robot inmediatamente, con intentos de reconexión y reintentos de envío

    global u esp32_ip esp32_port

    max_retries = 5;
    sent = false;

    for attempt = 1:max_retries
        try
            % Si no hay conexión activa, intenta reconectar
            if isempty(u)
                fprintf('[%d] No hay conexión activa. Intentando reconectar...\n', attempt);
                robot_connect();
                pause(0.1);
            end

            % Intentar enviar comando de parada
            robot_send_WiFi('stop');
            fprintf('¡Parada de emergencia enviada en el intento %d!\n', attempt);
            sent = true;
            break;

        catch ME
            warning('Error al intentar enviar el comando de parada (intento %d):\n%s', attempt, getReport(ME));
            pause(0.2);  % Pausa entre reintentos
        end
    end

    if ~sent
        warning('No se logró detener el robot tras %d intentos.', max_retries);
    end

    % Intentar desconectar
    try
        robot_disconnect();
        disp('Conexión cerrada después de la emergencia.');
    catch
        warning('No se pudo cerrar la conexión correctamente después de la parada.');
    end
end

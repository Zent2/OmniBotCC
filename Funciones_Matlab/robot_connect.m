function robot_connect()
    % Establece la conexión con el ESP32 vía UDP y guarda los datos

    global u esp32_ip esp32_port

    if isempty(u)
        esp32_ip = "192.168.50.222";  % Estos 2 datos están así en el ESP32 del robot
        esp32_port = 12345;

        try
            u = udpport("datagram", "IPV4");
            disp('Conexión UDP establecida correctamente.');
        catch ME
            disp('ERROR: No se pudo conectar al robot. Verifica que esté encendido.');
            disp(getReport(ME));
        end
    else
        disp('Ya existe una conexión UDP activa.');
    end
end

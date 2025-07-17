function robot_disconnect()
    % Cierra la conexión UDP y limpia variables globales

    global u esp32_ip esp32_port

    if ~isempty(u)
        try
            delete(u);  % Cierra la conexión
            disp('Conexión UDP cerrada correctamente.');
        catch ME
            disp('ERROR al cerrar la conexión:');
            disp(getReport(ME));
        end

        % Limpiar las variables globales
        clear global u esp32_ip esp32_port
    else
        disp('No hay una conexión UDP activa que cerrar.');
    end
end

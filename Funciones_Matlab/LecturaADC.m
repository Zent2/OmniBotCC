% CONFIGURACIÓN DEL PUERTO SERIAL
puerto = "COM3";
baud = 115200;
muestreo=20;
vel=402;
numLecturas = 60/(20/1000)*1.2;
% INICIALIZAR SERIAL
s = serialport(puerto, baud);
s.Timeout = 5;  % Ajuste del tiempo de espera
flush(s);
disp("Leyendo datos de un solo valor por línea...");

% VARIABLES
valores = zeros(numLecturas, 1);
tiempos = zeros(numLecturas, 1);

% INICIAR TIEMPO
tInicio = tic;

for i = 1:numLecturas
    try
        linea = readline(s);

        if ~ischar(linea) && ~isstring(linea)
            warning("Lectura inválida en línea %d: tipo de dato inesperado", i);
            continue;
        end

        valor = str2double(strtrim(linea));

        if isnan(valor)
            warning("Dato inválido en línea %d: %s", i, linea);
            continue;
        end

        valores(i) = valor;
        tiempos(i) = toc(tInicio);

    catch ME
        warning("Error en lectura %d: %s", i, ME.message);
        continue;
    end
end

% CREAR TABLA
tabla = table(tiempos, valores, 'VariableNames', {'Tiempo_s', 'ValorAnalogico'});

% GUARDAR EN EXCEL
filename = sprintf("analog_ESP32_datos%drpm.xlsx", vel);
writetable(tabla, filename);
disp("✅ Datos guardados en " + filename);
clear s;
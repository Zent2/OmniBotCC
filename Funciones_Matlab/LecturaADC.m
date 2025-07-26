% CONFIGURACIÓN DEL PUERTO SERIAL
puerto = "COM3";
baud = 115200;
numLecturas = 100;

% INICIALIZAR SERIAL
s = serialport(puerto, baud);
flush(s);
disp("Leyendo datos de un solo valor por línea...");

% VARIABLES
valores = zeros(numLecturas, 1);
tiempos = zeros(numLecturas, 1);

% INICIAR TIEMPO
tInicio = tic;

for i = 1:numLecturas
    linea = readline(s);
    valor = str2double(strtrim(linea));

    if isnan(valor)
        warning("Dato inválido en línea %d: %s", i, linea);
        continue;
    end

    valores(i) = valor;
    tiempos(i) = toc(tInicio);
end

% CREAR TABLA
tabla = table(tiempos, valores, 'VariableNames', {'Tiempo_s', 'ValorAnalogico'});

% GUARDAR EN EXCEL
filename = "analog_ESP32_datos.xlsx";
writetable(tabla, filename);
disp("✅ Datos guardados en " + filename);

%% INICIALIZACION DE LOS DATOS
% Cargar el archivo JSON
jsonFile = 'C:\Users\PC\Documents\Sensores y actuadores\DAQ\KartsGustavo.json'; % Nombre del archivo JSON

% Verifica si el archivo existe
if ~isfile(jsonFile)
    error('El archivo JSON no se encuentra en el directorio actual.');
end

% Abrir y leer el archivo
fid = fopen(jsonFile, 'r'); % Abrir el archivo en modo lectura
if fid == -1
    error('No se pudo abrir el archivo JSON.');
end

% Leer los datos
rawData = fread(fid, inf, 'uint8'); % Leer datos como uint8
fclose(fid); % Cerrar el archivo

% Convertir a cadena y decodificar el JSON
jsonData = char(rawData'); % Convertir a cadena de texto
dataStruct = jsondecode(jsonData); % Decodificar el JSON

% Acceder a los valores de 'payload' -> 'values'
valuesArray = dataStruct.payload.values;

% Verificar el tipo de la variable
disp('Tipo de variable:');
disp(class(valuesArray));

% Si ya es una matriz numérica, no es necesario convertirla
if isnumeric(valuesArray)
    disp('Los valores ya están en formato numérico.');
else
    % Si no es numérico, intentar convertir a matriz
    try
        valuesArray = cell2mat(valuesArray);
    catch
        error('No se pudo convertir los valores a una matriz numérica.');
    end
end

% Mostrar las primeras filas
disp('Primeras filas de los valores:');
disp(valuesArray(1:5, :)); % Muestra las primeras 5 filas

Acc = [valuesArray(:,1),valuesArray(:,2),valuesArray(:,3)];
Wa = [valuesArray(:,4),valuesArray(:,5),valuesArray(:,6)];

% Aceleraciones medidas
figure;
subplot(2,1,1);
plot(t, Acc(:,1), 'r'); hold on;
plot(t, Acc(:,2), 'g');
plot(t, Acc(:,3), 'b');
xlabel('Tiempo [s]');
ylabel('Aceleración medida [m/s^2]');
title('Aceleraciones medidas ');
legend('Aceleración X', 'Aceleración Y', 'Aceleración Z');
grid on;

% Velocidades medidas
subplot(2,1,2);
plot(t, Wa(:,1), 'r'); hold on;
plot(t, Wa(:,2), 'g');
plot(t, Wa(:,3), 'b');
xlabel('Tiempo [s]');
ylabel('Velocidad angular medidas [°/s]');
title('Velocidades angulares medidas');
legend('Velocidad angular X', 'Velocidad angular Y', 'Velocidad angular Z');
grid on;

%% FILTRO BUTTERWORTH
% Datos simulados 
fs = 1000; % Frecuencia de muestreo en Hz 

% Calcular el número de muestras
num_samples = length(Acc(:,1)); % Número de puntos en data (15,000 en este caso)

% Crear el vector de tiempo para que coincida con el tamaño de data
t = (0:num_samples-1) / fs; % Tiempo en segundos

% Parámetros del filtro
fc = 3; % Frecuencia de corte (Hz)
order = 5; % Orden del filtro

% Diseño del filtro Butterworth
[b, a] = butter(order, fc/(fs/2), 'low'); % Filtro pasa-bajas

% Aplicar el filtro a aceleraciones
Acc_f = [filtfilt(b, a, Acc(:,1)), filtfilt(b, a, Acc(:,2)), filtfilt(b, a, Acc(:,3))]

% Aplicar el filtro a velocidades angulares 

Wa_f = [filtfilt(b, a, Wa(:,1)), filtfilt(b, a, Wa(:,2)), filtfilt(b, a, Wa(:,3))]

% Aceleraciones filtradas
figure;
subplot(2,1,1);
plot(t, Acc_f(:,1), 'r'); hold on;
plot(t, Acc_f(:,2), 'g');
plot(t, Acc_f(:,3), 'b');
xlabel('Tiempo [s]');
ylabel('Aceleración filtrada [m/s^2]');
title('Aceleraciones filtradas');
legend('Aceleración X', 'Aceleración Y', 'Aceleración Z');
grid on;

% Graficar velocidades angulares filtradas
subplot(2,1,2);
plot(t, Wa_f(:,1), 'r'); hold on;
plot(t, Wa_f(:,2), 'g');
plot(t, Wa_f(:,3), 'b');
xlabel('Tiempo [s]');
ylabel('Velocidad angular filtrada [°/s]');
title('Velocidades angulares filtrad');
legend('Velocidad angular X', 'Velocidad angular Y', 'Velocidad angular Z');
grid on;

% Para deshacernos de las componentes que son perpendiculares al plano de
% interes y son paralelas a g, se sigue el siguiente procedimiento:
Acc_p = zeros(size(Acc_f));
Wa_p = zeros(size(Wa_f));
gravedad = [ 0 , 0 , -1];
% Quitar componentes de aceleraciones paralelas a la gravedad
for i = 1:size(Acc, 1)
    Acc_p(i, :) = Acc_f(i, :) - dot(gravedad, Acc_f(i, :)) * gravedad;  
end

% Quitar componentes velocidad paralelas a la gravedad
for i = 1:size(Wa, 1)
    Wa_p(i, :) = Wa_f(i, :) - dot(gravedad, Wa_f(i, :)) * gravedad; 
end

Acc_integrar = zeros(1,length(Acc_p(:,1))); %Vector para guardar componente de acc[x,y,z]

% Promedios de los datos
Accxpromedio = mean(Acc_p(:, 1));
Accypromedio = mean(Acc_p(:, 2));
Acczpromedio = mean(Acc_p(:, 3));

% Definir el vector de aceleración promedio unitario
Vector_direccionprom = [Accxpromedio, Accypromedio, Acczpromedio] / norm([Accxpromedio, Accypromedio, Acczpromedio]);
Vector_direccionprom(:,3) = 0;
%Acc_p(:,3) = 0; mandar a 0 componente 3 de las aceleraciones proyectadas

%vector_direcc = [escalar, escalar, 0]; * [acelpuntox, acelpuntoy, acelz] %Guarda la componente de cada punto de aceleración que esta orientada a la dirección promedio
for i = 1:(Acc_p(:,1))
   Acc_integrar(i) = dot(Acc_p(i,:),Vector_direccionprom); 
end

% Integrar para hallar magnitudes

velocidad_integrada = cumtrapz(t,Acc_integrar); % Velocidad
longitud_total = cumtrapz(t,velocidad_integrada); % Distancia recorrida a lo largo de una trayectoria curva (S) 

% Integrar componentes de las velocidades angulares

%pos_angularx = cumtrapz(t,Wa_p(:,1)); 
pos_angulary = cumtrapz(t,(Wa_p(:,2))); %Se usa posicion angular Y para hallar la trayectoria

diferencial_longitud = diff(longitud_total);

resultado=zeros(2,length(Acc_f(:,1)));
resultado(:,1)= [0;0];

for i=1:length(t)-1
    resultado(:,i+1)= resultado(:,i)+diferencial_longitud(i)*[cos(pos_angulary(i)) ; sin(pos_angulary(i))];
end   

figure;
plot(resultado(1,:),resultado(2,:))
title('Trayectoria Kart')
hold on;
grid on;

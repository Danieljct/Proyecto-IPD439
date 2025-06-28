% MATLAB Script para visualizar magnitudes FFT desde un archivo CSV

% --- Configuración ---
nombreArchivo = 'magnitudes_combinadas.csv'; % Asegúrate que esta ruta coincide con la de tu SD
Fs = 6660;                                % Frecuencia de muestreo del acelerómetro (Hz)
fft_points = 4096;                        % Número de puntos de la FFT utilizada en STM32

% Calcula el número de puntos de magnitud (N/2 + 1 para FFT real)
num_magnitudes = fft_points / 2 + 1;

% Define el eje de frecuencias
% Se extiende desde 0 hasta Fs/2 (frecuencia de Nyquist) con el número correcto de puntos.
frecuencias = linspace(0, Fs/2, num_magnitudes); % Hz

% --- Verificación y Carga del Archivo ---
if ~exist(nombreArchivo, 'file')
    error('Error: El archivo "%s" no se encuentra. Asegúrate de que la ruta es correcta y la SD está insertada.', nombreArchivo);
end

disp(['Leyendo el archivo: ' nombreArchivo]);

% Leer todo el archivo CSV. Cada fila representa un conjunto de magnitudes.
% Se asume que cada fila tiene 'num_magnitudes' columnas.
try
    datos_magnitudes = readmatrix(nombreArchivo);
catch ME
    error('Error al leer el archivo CSV. Asegúrate de que está correctamente formateado. Detalles: %s', ME.message);
end

% Verificar que las dimensiones son correctas
if size(datos_magnitudes, 2) ~= num_magnitudes
    error('Error: El número de columnas en el archivo CSV (%d) no coincide con el esperado de la FFT (%d).', size(datos_magnitudes, 2), num_magnitudes);
end

disp(['Archivo cargado. Total de ' num2str(size(datos_magnitudes, 1)) ' conjuntos de magnitudes.']);

% --- Configuración de la Ventana de la Figura ---
figure;
h_plot = plot(frecuencias, datos_magnitudes(1, :), 'b-'); % Plot inicial con la primera fila
grid on;
title('Magnitudes de la FFT (Animación Temporal)');
xlabel('Frecuencia (Hz)');
ylabel('Magnitud');
% Ajusta los límites del eje Y para que no cambie durante la animación,
% basándose en el valor máximo de todas las magnitudes.

xlim([0 Fs/2]); % Asegura el rango completo de frecuencias

% --- Animación Fila a Fila ---
disp('Iniciando animación. Presiona Ctrl+C en la ventana de comandos para detener.');
pause_duration = 0.05; % Duración de la pausa entre cada frame (segundos)

for i = 1:size(datos_magnitudes, 1)
    % Actualiza los datos del plot con la fila actual
    ylim_max = 6000;% max(datos_magnitudes(i,30:end)) * 1.1; % Un poco más del máximo para margen
    ylim([0 ylim_max]); % Siempre desde 0 hasta el máximo global
    set(h_plot, 'YData', datos_magnitudes(i, :));
    title(sprintf('Magnitudes de la FFT (Conjunto %d/%d)', i, size(datos_magnitudes, 1)));
    
    drawnow; % Fuerza la actualización del gráfico
    pause(pause_duration); % Pausa para la animación
    end

disp('Animación finalizada.');

% --- Guardar la animación como GIF ---
gif_filename = 'fft_animacion.gif';
disp(['Guardando animación como GIF: ' gif_filename]);

figure;
h_plot = plot(frecuencias, datos_magnitudes(1, :), 'b-');
grid on;
title('Magnitudes de la FFT (Animación Temporal)');
xlabel('Frecuencia (Hz)');
ylabel('Magnitud');
xlim([0 Fs/2]);
ylim_max = 6000;
ylim([0 ylim_max]);

for i = 1:size(datos_magnitudes, 1)
    set(h_plot, 'YData', datos_magnitudes(i, :));
    title(sprintf('Magnitudes de la FFT (Conjunto %d/%d)', i, size(datos_magnitudes, 1)));
    drawnow;
    
    frame = getframe(gcf);
    im = frame2im(frame);
    [A,map] = rgb2ind(im,256);
    if i == 1
        imwrite(A,map,gif_filename,'gif','LoopCount',Inf,'DelayTime',pause_duration);
    else
        imwrite(A,map,gif_filename,'gif','WriteMode','append','DelayTime',pause_duration);
    end
end

disp('GIF guardado exitosamente.');

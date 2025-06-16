import sys
import serial
import threading
import queue
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


class SerialReader(threading.Thread):
    """
    Clase para leer datos del puerto serial en un hilo separado.
    Esto evita que la interfaz de usuario se congele mientras se esperan datos.
    """
    def __init__(self, port, baudrate, data_queue):
        super().__init__()
        self.serial = serial.Serial(port, baudrate, timeout=1)
        self.data_queue = data_queue
        self.running = True

    def run(self):
        """
        Método principal del hilo que lee continuamente del puerto serial.
        Decodifica la línea, la divide en valores enteros y los pone en la cola.
        """
        while self.running:
            try:
                # Lee una línea del puerto serial, la decodifica y elimina espacios
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                # Convierte los valores de la línea a enteros
                values = list(map(int, line.split()))
                if len(values) == 4: # Esperamos 4 valores (timestamp, X, Y, Z)
                    self.data_queue.put(values)
            except Exception:
                # Si ocurre un error (ej. línea incompleta, datos no numéricos),
                # simplemente continúa intentando leer la siguiente línea.
                continue

    def stop(self):
        """
        Detiene el hilo de lectura y cierra el puerto serial de forma segura.
        """
        self.running = False
        self.serial.close()


class SerialPlotter(QtWidgets.QMainWindow):
    """
    Clase principal de la aplicación PyQt5 para graficar datos seriales en tiempo real.
    Muestra la aceleración en el tiempo y la FFT cuando está pausado.
    """
    def __init__(self, port='COM3', baudrate=115200, axis='X'):
        """
        Inicializa la ventana principal de la aplicación.
        :param port: Puerto serial a conectar (ej. 'COM3', '/dev/ttyUSB0').
        :param baudrate: Velocidad de transmisión del puerto serial.
        :param axis: Eje del acelerómetro a graficar ('X', 'Y', 'Z').
        """
        super().__init__()
        self.setWindowTitle("Acelerómetro en tiempo real")
        self.resize(800, 700)

        # Almacena el nombre del eje para mostrar en los títulos de los gráficos
        self.axis_name = axis
        # Mapea el nombre del eje a su índice en la lista de valores [timestamp, X, Y, Z]
        self.axis_index = {'X': 0, 'Y': 1, 'Z': 2}[axis]
        self.data = np.zeros(6000) # Buffer para almacenar los datos a graficar
        self.data_queue = queue.Queue() # Cola para recibir datos del hilo serial
        self.paused = False # Estado de pausa de la visualización

        # --- Parámetros de filtrado de datos ---
        # Define el rango de valores válidos para la aceleración del sensor.
        # Cualquier valor recibido fuera de este rango será considerado erróneo y filtrado.
        # Ajusta estos límites según el rango típico y esperado de tu sensor.
        self.min_valid_acceleration = -30000 # Límite inferior para valores válidos
        self.max_valid_acceleration = 30000  # Límite superior para valores válidos

        # --- Nuevo parámetro de filtrado: Máximo cambio permitido entre muestras ---
        # Este valor define la diferencia máxima absoluta permitida entre un nuevo dato
        # y el último dato válido recibido. Si la diferencia es mayor, el nuevo dato
        # se considera atípico (un "salto") y se filtra.
        # Ajusta este valor (ej. 8000 si tus datos suelen ser 8000 y 16000 es un error).
        self.max_delta_acceleration = 6000 # Máximo cambio permitido entre una muestra y la anterior


        # Hilo de lectura del puerto serial
        self.reader = SerialReader(port, baudrate, self.data_queue)
        self.reader.start()

        # --- Configuración de los widgets de gráficos (PyQtGraph) ---
        # Gráfico principal de la aceleración en el tiempo
        self.plot_widget = pg.PlotWidget(title=f"Aceleración en el tiempo (Eje {self.axis_name})")
        self.plot = self.plot_widget.plot(self.data, pen='c') # 'c' para color cian
        self.plot_widget.setLabel('left', 'Aceleración')
        self.plot_widget.setLabel('bottom', 'Muestras')
        # Establece el rango Y del gráfico para que coincida con los límites de filtrado,
        # lo que ayuda a visualizar los datos dentro del rango esperado.
        self.plot_widget.setYRange(self.min_valid_acceleration, self.max_valid_acceleration)
        self.plot_widget.showGrid(x=True, y=True) # Muestra la cuadrícula para facilitar la lectura

        # Gráfico para la Transformada Rápida de Fourier (FFT)
        self.fft_widget = pg.PlotWidget(title=f"FFT (Frecuencia) - Eje {self.axis_name}")
        self.fft_plot = self.fft_widget.plot([], pen='m') # 'm' para color magenta
        self.fft_widget.setLabel('left', 'Magnitud')
        self.fft_widget.setLabel('bottom', 'Frecuencia (Hz)')
        self.fft_widget.hide() # Oculta el gráfico FFT inicialmente

        # Conexiones para el seguimiento del ratón en los gráficos
        # Corrección: El signal sigMouseMoved emite directamente un QPointF.
        self.plot_widget.scene().sigMouseMoved.connect(self.on_mouse_moved_plot)
        self.fft_widget.scene().sigMouseMoved.connect(self.on_mouse_moved_fft)

        # Botón para pausar/reanudar la visualización y activar/desactivar la FFT
        self.pause_button = QtWidgets.QPushButton("Pausar")
        self.pause_button.clicked.connect(self.toggle_pause)

        # --- Configuración del layout principal de la ventana ---
        central_widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.plot_widget)
        layout.addWidget(self.fft_widget)
        layout.addWidget(self.pause_button)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Timer para actualizar el gráfico periódicamente
        # Se actualiza aproximadamente cada 16 milisegundos (aprox. 60 FPS)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(16)

    def toggle_pause(self):
        """
        Alterna el estado de pausa de la visualización.
        Cuando se pausa, calcula y muestra la FFT.
        Cuando se reanuda, oculta la FFT y limpia la cola de datos.
        """
        self.paused = not self.paused
        self.pause_button.setText("Reanudar" if self.paused else "Pausar")

        if self.paused:
            # Si está pausado, calcula y muestra la Transformada Rápida de Fourier
            self.plot_fft()
            self.fft_widget.show()
            # Ajusta el rango Y del gráfico principal para mostrar todo el rango de datos capturados
            # cuando está pausado, lo que puede ser útil para un análisis detallado.
            self.plot_widget.setYRange(np.min(self.data), np.max(self.data))
        else:
            # Si se reanuda, oculta la FFT
            self.fft_widget.hide()
            # Limpia la cola de datos para evitar que se procesen datos acumulados
            # durante la pausa, lo que podría causar un "salto" repentino en la gráfica.
            with self.data_queue.mutex:
                self.data_queue.queue.clear()
            # Restablece el rango Y del gráfico principal a los límites de filtrado
            self.plot_widget.setYRange(self.min_valid_acceleration, self.max_valid_acceleration)


    def plot_fft(self):
        """
        Calcula y grafica la Transformada Rápida de Fourier (FFT) de los datos actuales.
        """
        fs = 3330  # Frecuencia de muestreo en Hz (ajusta esto a la tasa de tu sensor)
        N = len(self.data) # Número de puntos de datos

        # Centra los datos eliminando su media. Esto es importante para un cálculo preciso de la FFT.
        data_centered = self.data - np.mean(self.data)

        # Aplica una ventana de Hann a los datos. Esto ayuda a reducir el "leakage" espectral,
        # que son artefactos en la FFT causados por la longitud finita de la señal.
        window = np.hanning(N)
        fft = np.fft.rfft(data_centered * window) # Calcula la FFT para señales reales (más eficiente)
        freqs = np.fft.rfftfreq(N, d=1/fs) # Calcula las frecuencias correspondientes a los bins de la FFT
        magnitude = np.abs(fft) / (N / 2)  # Calcula la magnitud de la FFT y la escala correctamente

        self.fft_plot.setData(freqs, magnitude)


    def update_plot(self):
        """
        Actualiza el gráfico con los nuevos datos recibidos del puerto serial.
        Incorpora la lógica de filtrado de datos erróneos por rango absoluto y por cambio brusco.
        """
        if self.paused:
            return # Si la visualización está pausada, no se actualiza el gráfico.

        # Procesa todos los datos que se han acumulado en la cola desde la última actualización.
        while not self.data_queue.empty():
            values = self.data_queue.get() # Obtiene el siguiente conjunto de valores del sensor
            acc_value = values[self.axis_index] # Extrae el valor de aceleración del eje seleccionado

            # --- Lógica de filtrado de datos erróneos ---
            # 1. Filtro por rango absoluto:
            # Si el valor de aceleración está fuera del rango de valores válidos definidos,
            # se considera un dato erróneo y se ignora. Esto evita que picos incorrectos
            # o datos corruptos se muestren en el gráfico.
            if not (self.min_valid_acceleration <= acc_value <= self.max_valid_acceleration):
                print(f"Valor atípico detectado y filtrado por rango absoluto en el eje {self.axis_name}: {acc_value}. Se mantiene el valor anterior en la gráfica.")
                continue # Pasa a la siguiente muestra en la cola si este valor es descartado

            # 2. Filtro por cambio brusco (delta filter):
            # Este filtro solo se aplica si ya hay al menos un dato válido en el array,
            # ya que necesita una referencia (el último valor válido) para comparar.
            # Se evita aplicar este filtro cuando el buffer está lleno de ceros iniciales
            # o al inicio de la captura de datos reales.
            if self.data.size > 0 and self.data[-1] != 0: # Asegura que hay un valor anterior significativo
                delta = abs(acc_value - self.data[-1])
                if delta > self.max_delta_acceleration:
                    print(f"Valor atípico detectado y filtrado por cambio excesivo en el eje {self.axis_name}: {acc_value} (cambio: {delta}). Se mantiene el valor anterior en la gráfica.")
                    continue # Pasa a la siguiente muestra en la cola si este valor es descartado

            # Si el valor pasó todos los filtros, se añade al buffer de datos.
            # Se desplazan los datos existentes y se inserta el nuevo valor al final.
            self.data = np.roll(self.data, -1)
            self.data[-1] = acc_value

        self.plot.setData(self.data) # Actualiza la visualización del gráfico con los datos procesados.


    def closeEvent(self, event):
        """
        Método que se ejecuta cuando se cierra la ventana.
        Asegura que el hilo de lectura serial se detenga correctamente.
        """
        self.reader.stop()
        event.accept()

    def on_mouse_moved_plot(self, evt):
        """
        Maneja el evento de movimiento del ratón sobre el gráfico de tiempo.
        Imprime las coordenadas (muestra y valor de aceleración) en la consola.
        """
        # Corrección: evt ya es el QPointF, no necesita .pos()
        pos = evt 
        # Verifica que la posición del ratón esté dentro de los límites del widget del gráfico
        if self.plot_widget.sceneBoundingRect().contains(pos):
            # Convierte las coordenadas de la escena a coordenadas del gráfico (tiempo, aceleración)
            mouse_point = self.plot_widget.plotItem.vb.mapSceneToView(pos)
            x = int(mouse_point.x()) # Muestra (índice en el array de datos)
            if 0 <= x < len(self.data):
                y = self.data[x] # Valor de aceleración en esa muestra
                print(f"Gráfico de Tiempo - Muestra {x}: {y:.2f}")

    def on_mouse_moved_fft(self, evt):
        """
        Maneja el evento de movimiento del ratón sobre el gráfico FFT.
        Imprime las coordenadas (frecuencia y magnitud) en la consola.
        """
        if not self.paused:
            return  # La FFT solo es relevante si el gráfico está pausado
        # Corrección: evt ya es el QPointF, no necesita .pos()
        pos = evt 
        # Verifica que la posición del ratón esté dentro de los límites del widget del gráfico FFT
        if self.fft_widget.sceneBoundingRect().contains(pos):
            # Convierte las coordenadas de la escena a coordenadas del gráfico (frecuencia, magnitud)
            mouse_point = self.fft_widget.plotItem.vb.mapSceneToView(pos)
            x = mouse_point.x() # Frecuencia
            y = mouse_point.y() # Magnitud
            print(f"Gráfico FFT - Frecuencia: {x:.2f} Hz | Magnitud: {y:.2f}")


if __name__ == '__main__':
    # --- Configuración de la aplicación al iniciar ---
    app = QtWidgets.QApplication(sys.argv)

    # Configura el puerto serial, el eje a visualizar y la velocidad de comunicación.
    # ¡IMPORTANTE!: Asegúrate de que 'port' y 'baudrate' coincidan con la configuración de tu sensor.
    # Puedes encontrar el puerto correcto en tu sistema operativo (ej. Administrador de dispositivos en Windows).
    port = 'COM18'      # Ejemplo: 'COM3' en Windows, o '/dev/ttyUSB0' en Linux
    axis = 'Z'          # Elige el eje a graficar: 'X', 'Y', o 'Z'
    baudrate = 2000000  # Velocidad de comunicación del puerto serial (ej. 115200, 2000000)

    # Crea una instancia de la ventana principal y la muestra
    window = SerialPlotter(port=port, baudrate=baudrate, axis=axis)
    window.show()

    # Inicia el bucle de eventos de la aplicación PyQt5
    sys.exit(app.exec_())

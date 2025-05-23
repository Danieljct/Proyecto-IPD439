import sys
import serial
import threading
import queue
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


class SerialReader(threading.Thread):
    def __init__(self, port, baudrate, data_queue):
        super().__init__()
        self.serial = serial.Serial(port, baudrate, timeout=1)
        self.data_queue = data_queue
        self.running = True

    def run(self):
        while self.running:
            try:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                values = list(map(int, line.split()))
                if len(values) == 4:
                    self.data_queue.put(values)
            except Exception:
                continue

    def stop(self):
        self.running = False
        self.serial.close()


class SerialPlotter(QtWidgets.QMainWindow):
    def __init__(self, port='COM3', baudrate=115200, axis='X'):
        super().__init__()
        self.setWindowTitle("Acelerómetro en tiempo real")
        self.resize(800, 700)

        self.axis_index = {'X': 0, 'Y': 1, 'Z': 2}[axis]
        self.data = np.zeros(6000)
        self.data_queue = queue.Queue()
        self.paused = False

        # Hilo de lectura
        self.reader = SerialReader(port, baudrate, self.data_queue)
        self.reader.start()

        # Widgets de gráficos
        self.plot_widget = pg.PlotWidget(title="Aceleración en el tiempo")
        self.plot = self.plot_widget.plot(self.data, pen='c')
        self.plot_widget.setLabel('left', 'Aceleración')
        self.plot_widget.setLabel('bottom', 'Muestras')
        self.plot_widget.setYRange(-10000, 10000)

        self.fft_widget = pg.PlotWidget(title="FFT (Frecuencia)")
        self.fft_plot = self.fft_widget.plot([], pen='m')
        self.fft_widget.setLabel('left', 'Magnitud')
        self.fft_widget.setLabel('bottom', 'Frecuencia (Hz)')
        self.fft_widget.hide()
        
        self.plot_widget.scene().sigMouseMoved.connect(self.on_mouse_moved_plot)
        self.fft_widget.scene().sigMouseMoved.connect(self.on_mouse_moved_fft)
        # Botón
        self.pause_button = QtWidgets.QPushButton("Pausar")
        self.pause_button.clicked.connect(self.toggle_pause)

        # Layout
        central_widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.plot_widget)
        layout.addWidget(self.fft_widget)
        layout.addWidget(self.pause_button)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(16)

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.setText("Reanudar" if self.paused else "Pausar")

        if self.paused:
            self.plot_fft()
            self.fft_widget.show()
        else:
            self.fft_widget.hide()
            with self.data_queue.mutex:
                self.data_queue.queue.clear()  # <--- Limpia la cola



    def plot_fft(self):
        fs = 3330  # Frecuencia de muestreo en Hz
        N = len(self.data)
        data_centered = self.data - np.mean(self.data)

        # FFT con ventana de Hann para reducir leakage
        window = np.hanning(N)
        fft = np.fft.rfft(data_centered * window)
        freqs = np.fft.rfftfreq(N, d=1/fs)
        magnitude = np.abs(fft) / (N / 2)  # Escalado correcto

        self.fft_plot.setData(freqs, magnitude)
        print(f"Frecuencia máxima mostrada: {freqs[-1]:.1f} Hz")


    def update_plot(self):
        if self.paused:
            return

        while not self.data_queue.empty():
            values = self.data_queue.get()
            acc_value = values[self.axis_index]

            # Verifica que el nuevo valor no sea igual al último para evitar duplicados
            if acc_value != self.data[-1]:
                self.data = np.roll(self.data, -1)
                self.data[-1] = acc_value

        self.plot.setData(self.data)


    def closeEvent(self, event):
        self.reader.stop()
        event.accept()

    def on_mouse_moved_plot(self, evt):
        pos = evt
        if self.plot_widget.sceneBoundingRect().contains(pos):
            mouse_point = self.plot_widget.plotItem.vb.mapSceneToView(pos)
            x = int(mouse_point.x())
            if 0 <= x < len(self.data):
                y = self.data[x]
                print(f"Tiempo - Muestra {x}: {y:.2f}")

    def on_mouse_moved_fft(self, evt):
        if not self.paused:
            return  # FFT solo tiene sentido si está pausado
        pos = evt
        if self.fft_widget.sceneBoundingRect().contains(pos):
            mouse_point = self.fft_widget.plotItem.vb.mapSceneToView(pos)
            x = mouse_point.x()
            y = mouse_point.y()
            print(f"Frecuencia: {x:.2f} Hz | Magnitud: {y:.2f}")


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)

    # Configura aquí
    port = 'COM18'     # o '/dev/ttyUSB0'
    axis = 'Z'        # 'X', 'Y', 'Z'
    baudrate = 2000000

    window = SerialPlotter(port=port, baudrate=baudrate, axis=axis)
    window.show()

    sys.exit(app.exec_())

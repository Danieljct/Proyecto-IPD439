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
        self.resize(800, 550)

        self.axis_index = {'X': 0, 'Y': 1, 'Z': 2}[axis]
        self.data = np.zeros(3000)
        self.data_queue = queue.Queue()
        self.paused = False

        # Iniciar hilo de lectura
        self.reader = SerialReader(port, baudrate, self.data_queue)
        self.reader.start()

        # Crear widgets
        self.plot_widget = pg.PlotWidget()
        self.plot = self.plot_widget.plot(self.data, pen='c')
        self.plot_widget.setLabel('left', 'Aceleración')
        self.plot_widget.setLabel('bottom', 'Muestras')
        self.plot_widget.setYRange(-10000, 10000)

        self.pause_button = QtWidgets.QPushButton("Pausar")
        self.pause_button.clicked.connect(self.toggle_pause)

        # Layout vertical
        central_widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.plot_widget)
        layout.addWidget(self.pause_button)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Timer de actualización
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(16)

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.setText("Reanudar" if self.paused else "Pausar")

    def update_plot(self):
        if self.paused:
            return
        while not self.data_queue.empty():
            values = self.data_queue.get()
            acc_value = values[self.axis_index]
            self.data = np.roll(self.data, -1)
            self.data[-1] = acc_value
            self.plot.setData(self.data)

    def closeEvent(self, event):
        self.reader.stop()
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)

    # Configura tu puerto y eje aquí
    port = 'COM18'     # o '/dev/ttyUSB0' en Linux
    axis = 'Z'        # 'X', 'Y', 'Z'
    baudrate = 2000000

    window = SerialPlotter(port=port, baudrate=baudrate, axis=axis)
    window.show()

    sys.exit(app.exec_())

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
                value = float(line)  # Acepta flotantes también
                self.data_queue.put(value)
            except Exception:
                continue

    def stop(self):
        self.running = False
        self.serial.close()


class SerialPlotter(QtWidgets.QMainWindow):
    def __init__(self, port='COM3', baudrate=115200):
        super().__init__()
        self.setWindowTitle("Gráfico UART en tiempo real")
        self.resize(800, 600)

        self.data = np.zeros(10000)
        self.data_queue = queue.Queue()

        self.reader = SerialReader(port, baudrate, self.data_queue)
        self.reader.start()

        # Widget de gráfico
        self.plot_widget = pg.PlotWidget(title="Datos UART")
        self.plot = self.plot_widget.plot(self.data, pen='y')
        self.plot_widget.setLabel('left', 'Valor')
        self.plot_widget.setLabel('bottom', 'Muestras')

        # Layout
        central_widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.plot_widget)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Timer para actualizar
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(16)

    def update_plot(self):
        while not self.data_queue.empty():
            value = self.data_queue.get()
            self.data = np.roll(self.data, -1)
            self.data[-1] = value
        self.plot.setData(self.data)

    def closeEvent(self, event):
        self.reader.stop()
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)

    port = 'COM18'       # Cambia esto
    baudrate = 2000000   # Cambia esto también si es necesario

    window = SerialPlotter(port=port, baudrate=baudrate)
    window.show()

    sys.exit(app.exec_())

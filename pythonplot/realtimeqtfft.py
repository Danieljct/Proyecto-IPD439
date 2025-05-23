import serial
import struct
import time
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore

# Configuración del puerto serial
SERIAL_PORT = 'COM18'
BAUD_RATE = 2000000

start_marker = b'DMA x:'
end_marker = b'END.'
buffer = bytearray()
fs = 6660  # Frecuencia de muestreo en Hz

# Configurar ventana
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="FFT en Tiempo Real")
win.show()
win.setWindowTitle('FFT Viewer')

plot = win.addPlot(title="FFT en Tiempo Real")
plot.setLabel('bottom', 'Frecuencia', units='Hz')
plot.setLabel('left', 'Magnitud')
curve = plot.plot(pen='y')
plot.setYRange(0, 1000)

# Label para mostrar coordenadas
coord_label = pg.LabelItem(justify='right')
win.addItem(coord_label)

# Manejo de coordenadas del mouse
def mouseMoved(evt):
    pos = evt[0]  # evento del mouse
    if plot.sceneBoundingRect().contains(pos):
        mouse_point = plot.vb.mapSceneToView(pos)
        x = mouse_point.x()
        y = mouse_point.y()
        coord_label.setText(f"<span style='font-size: 14pt'>f = {x:.1f} Hz,&nbsp;&nbsp; Mag = {y:.1f}</span>")

proxy = pg.SignalProxy(plot.scene().sigMouseMoved, rateLimit=60, slot=mouseMoved)

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Puerto serial {SERIAL_PORT} abierto correctamente.")

    def update():
        global buffer
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            buffer.extend(data)

            while start_marker in buffer and end_marker in buffer:
                start_index = buffer.find(start_marker)
                end_index = buffer.find(end_marker, start_index)

                if end_index == -1 or end_index < start_index:
                    break

                raw_data = buffer[start_index + len(start_marker):end_index]
                buffer = buffer[end_index + len(end_marker):]

                if len(raw_data) % 4 != 0:
                    print("Tamaño inválido, descartando paquete.")
                    continue

                try:
                    data_values = struct.unpack(f'{len(raw_data)//4}f', raw_data)
                    n = len(data_values)
                    freqs = np.linspace(0, fs / 2, n)

                    curve.setData(freqs, data_values)

                    plot.setYRange(0, max(data_values[10:2048]) * 1.1 if data_values else 1)

                except struct.error as e:
                    print(f"Error al desempaquetar: {e}")

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(5)

    app.exec_()

except serial.SerialException as e:
    print(f"Error de puerto serial: {e}")
except KeyboardInterrupt:
    print("Programa terminado por el usuario.")
finally:
    if 'ser' in locals() and ser.isOpen():
        ser.close()
        print("Puerto serial cerrado.")

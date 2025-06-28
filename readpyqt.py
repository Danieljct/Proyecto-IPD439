import sys
import os
import numpy as np
import pandas as pd
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QWidget,
                             QPushButton, QLabel, QHBoxLayout, QFileDialog, QMessageBox, QSlider, QStyle) 
from PyQt5.QtCore import QTimer, Qt # <--- QTimer added here
from PyQt5.QtGui import QIcon
import pyqtgraph as pg

class FFTViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Visualizador de Magnitudes FFT')
        self.setGeometry(100, 100, 1000, 700)
        self.setWindowIcon(QIcon(self.style().standardIcon(QStyle.SP_ComputerIcon))) # Generic icon

        self.filename = 'magnitudes_combinadas.csv'  # Default filename
        self.fs = 6660  # Sampling frequency (Hz)
        self.fft_points = 4096  # Number of FFT points used in STM32
        self.num_magnitudes = int(self.fft_points / 2 + 1)
        self.frequencies = np.linspace(0, self.fs / 2, self.num_magnitudes)

        self.data_magnitudes = None
        self.current_frame = 0
        self.animation_timer = QTimer(self)
        self.animation_timer.timeout.connect(self.update_plot)
        self.pause_duration_ms = 50  # Default pause duration in milliseconds

        self.init_ui()
        self.load_initial_file()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # --- Top controls (File selection, Play/Pause) ---
        control_layout = QHBoxLayout()

        self.load_button = QPushButton('Cargar Archivo CSV')
        self.load_button.clicked.connect(self.open_file_dialog)
        control_layout.addWidget(self.load_button)

        self.play_pause_button = QPushButton('Reproducir')
        self.play_pause_button.clicked.connect(self.toggle_animation)
        self.play_pause_button.setEnabled(False)  # Disabled until data is loaded
        control_layout.addWidget(self.play_pause_button)

        self.stop_button = QPushButton('Detener')
        self.stop_button.clicked.connect(self.stop_animation)
        self.stop_button.setEnabled(False) # Disabled until animation starts
        control_layout.addWidget(self.stop_button)

        self.speed_label = QLabel('Velocidad:')
        control_layout.addWidget(self.speed_label)

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(10)
        self.speed_slider.setMaximum(200)
        self.speed_slider.setValue(self.pause_duration_ms)
        self.speed_slider.setSingleStep(10)
        self.speed_slider.setTickInterval(10)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.valueChanged.connect(self.update_animation_speed)
        control_layout.addWidget(self.speed_slider)

        self.frame_label = QLabel('Frame: 0/0')
        control_layout.addWidget(self.frame_label)

        main_layout.addLayout(control_layout)

        # --- Plotting area ---
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        self.plot_widget.setTitle("Magnitudes de la FFT (Animación Temporal)")
        self.plot_widget.setLabel('left', "Magnitud")
        self.plot_widget.setLabel('bottom', "Frecuencia (Hz)")
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setXRange(0, self.fs / 2)
        self.plot_curve = self.plot_widget.plot(self.frequencies, np.zeros(self.num_magnitudes), pen='b')
        main_layout.addWidget(self.plot_widget)

    def load_initial_file(self):
        """Attempts to load the default CSV file on startup."""
        if os.path.exists(self.filename):
            self.load_csv_data(self.filename)
        else:
            QMessageBox.warning(self, "Archivo no encontrado",
                                f"El archivo '{self.filename}' no se encuentra en la misma ubicación que el script. "
                                "Por favor, selecciona el archivo manualmente.")
            self.open_file_dialog()

    def open_file_dialog(self):
        """Opens a file dialog to select the CSV file."""
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getOpenFileName(self, "Seleccionar archivo CSV", "",
                                                   "Archivos CSV (*.csv);;Todos los archivos (*)", options=options)
        if file_path:
            self.filename = file_path
            self.load_csv_data(self.filename)

    def load_csv_data(self, file_path):
        """Loads data from the specified CSV file."""
        try:
            # --- MODIFICATION STARTS HERE ---
            # Option 1: If your CSV has a header, skip it and then read without header inference
            # df = pd.read_csv(file_path, skiprows=1, header=None)

            # Option 2: If your CSV has NO header (as was intended by original code)
            # but might have mixed types or non-numeric entries, use errors='coerce'
            df = pd.read_csv(file_path, header=None)
            
            # Apply to_numeric with coerce to handle any non-numeric entries by turning them into NaN
            # This is crucial for the "ufunc 'isfinite' not supported" error
            df = df.apply(pd.to_numeric, errors='coerce')
            
            # Now, if you expect only numbers, you might want to drop rows/columns with NaNs
            # Or, for plotting, `pyqtgraph` and `numpy.nanmax` (see below) handle NaNs gracefully.
            
            self.data_magnitudes = df.values
            # --- MODIFICATION ENDS HERE ---

            if self.data_magnitudes.shape[1] != self.num_magnitudes:
                QMessageBox.critical(self, "Error de Formato",
                                     f"El número de columnas en el archivo CSV ({self.data_magnitudes.shape[1]}) "
                                     f"no coincide con el esperado de la FFT ({self.num_magnitudes}).")
                self.data_magnitudes = None
                self.play_pause_button.setEnabled(False)
                return

            # Check if there are any non-numeric values (now NaNs) and warn the user
            if np.isnan(self.data_magnitudes).any():
                QMessageBox.warning(self, "Advertencia de Datos",
                                    "El archivo CSV contiene valores no numéricos que se han convertido a NaN (Not a Number). "
                                    "Esto podría afectar la visualización.")

            QMessageBox.information(self, "Archivo Cargado",
                                    f"Archivo '{os.path.basename(file_path)}' cargado exitosamente. "
                                    f"Total de {self.data_magnitudes.shape[0]} conjuntos de magnitudes.")
            self.current_frame = 0
            self.plot_initial_frame()
            self.play_pause_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.play_pause_button.setText('Reproducir')
            self.frame_label.setText(f'Frame: {self.current_frame + 1}/{self.data_magnitudes.shape[0]}')

        except Exception as e:
            QMessageBox.critical(self, "Error al Leer CSV",
                                 f"Error al leer el archivo CSV. Asegúrate de que está correctamente formateado.\nDetalles: {e}")
            self.data_magnitudes = None
            self.play_pause_button.setEnabled(False)

    def plot_initial_frame(self):
        """Plots the first frame of the data."""
        if self.data_magnitudes is not None and self.data_magnitudes.shape[0] > 0:
            self.plot_curve.setData(self.frequencies, self.data_magnitudes[0, :])
            # Set initial Y-axis range based on the first frame or global max
            # Use np.nanmax to handle potential NaN values
            ylim_max = np.nanmax(self.data_magnitudes[0, 30:]) * 1.1 if self.data_magnitudes[0, 30:].size > 0 and not np.all(np.isnan(self.data_magnitudes[0, 30:])) else 1
            self.plot_widget.setYRange(0, ylim_max)
        else:
            self.plot_curve.setData(self.frequencies, np.zeros(self.num_magnitudes)) # Clear plot if no data

    def update_plot(self):
        """Updates the plot with the next frame of data for animation."""
        if self.data_magnitudes is None or self.current_frame >= self.data_magnitudes.shape[0]:
            self.stop_animation()
            return

        current_magnitudes = self.data_magnitudes[self.current_frame, :]
        self.plot_curve.setData(self.frequencies, current_magnitudes)

        # Dynamically adjust Y-axis limit for the current frame
        # Use np.nanmax to handle potential NaN values
        ylim_max = np.nanmax(current_magnitudes[30:]) * 1.1 if current_magnitudes[30:].size > 0 and not np.all(np.isnan(current_magnitudes[30:])) else 1
        self.plot_widget.setYRange(0, ylim_max)

        self.plot_widget.setTitle(f"Magnitudes de la FFT (Conjunto {self.current_frame + 1}/{self.data_magnitudes.shape[0]})")
        self.frame_label.setText(f'Frame: {self.current_frame + 1}/{self.data_magnitudes.shape[0]}')

        self.current_frame += 1
        if self.current_frame >= self.data_magnitudes.shape[0]:
            self.current_frame = 0 # Loop back to start

    def toggle_animation(self):
        """Starts or pauses the animation."""
        if self.animation_timer.isActive():
            self.animation_timer.stop()
            self.play_pause_button.setText('Reproducir')
            self.stop_button.setEnabled(False)
        else:
            if self.data_magnitudes is not None:
                self.animation_timer.start(self.pause_duration_ms)
                self.play_pause_button.setText('Pausar')
                self.stop_button.setEnabled(True)
                # If at the end, restart from the beginning
                if self.current_frame >= self.data_magnitudes.shape[0]:
                    self.current_frame = 0
                    self.plot_initial_frame() # Re-plot the first frame to reset the Y-axis range potentially

    def stop_animation(self):
        """Stops the animation and resets to the first frame."""
        self.animation_timer.stop()
        self.play_pause_button.setText('Reproducir')
        self.stop_button.setEnabled(False)
        self.current_frame = 0
        if self.data_magnitudes is not None:
            self.plot_initial_frame() # Plot the first frame
            self.frame_label.setText(f'Frame: {self.current_frame + 1}/{self.data_magnitudes.shape[0]}')
        else:
            self.frame_label.setText('Frame: 0/0')


    def update_animation_speed(self, value):
        """Updates the animation speed based on slider value."""
        self.pause_duration_ms = value
        if self.animation_timer.isActive():
            self.animation_timer.start(self.pause_duration_ms) # Restart timer with new interval

# --- Main application execution ---
if __name__ == '__main__':
    # Create a dummy magnitudes.csv for testing if it doesn't exist
    if not os.path.exists('magnitudes_combinadas.csv'):
        print("Creating a dummy 'magnitudes_combinadas.csv' for demonstration.")
        dummy_data = np.random.rand(100, 2049) * 100 # 100 frames, 2049 magnitudes
        pd.DataFrame(dummy_data).to_csv('magnitudes_combinadas.csv', header=False, index=False)
        print("Dummy file created. You can replace it with your actual data.")

    app = QApplication(sys.argv)
    viewer = FFTViewer()
    viewer.show()
    sys.exit(app.exec_())
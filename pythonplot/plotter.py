import serial
import matplotlib.pyplot as plt
import re
import numpy as np
import time

# Configuración del puerto serial
# Reemplaza 'COM3' con el nombre de tu puerto serial
# Reemplaza 9600 con la velocidad en baudios de tu comunicación UART
SERIAL_PORT = 'COM18'
BAUD_RATE = 2000000

# Configurar la gráfica interactiva
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], 'o-') # Usamos 'o-' para puntos y líneas

# Configurar los ejes
ax.set_xlabel("Índice del Dato")
ax.set_ylabel("Valor del Dato")
ax.set_title("Datos de DMA en Tiempo Real")
ax.grid(True)

buffer = ""

try:
    # Abrir el puerto serial
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Puerto serial {SERIAL_PORT} abierto correctamente.")

    while True:
        if ser.in_waiting > 0:
            # Leer datos del puerto serial
            data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            buffer += data

            # Procesar el buffer línea por línea o por sets de datos
            while "FIFO reseteada." in buffer:
                end_index = buffer.find("FIFO reseteada.")
                set_data_str = buffer[:end_index]
                buffer = buffer[end_index + len("FIFO reseteada."):]

                # Buscar el inicio del set de datos
                start_index = set_data_str.find("DMA x:")
                if start_index != -1:
                    data_values_str = set_data_str[start_index + len("DMA x:"):].strip()
                    # Extraer los números
                    try:
                        data_values = [int(val) for val in data_values_str.split()]

                        if data_values:
                            # Actualizar la gráfica
                            ax.clear() # Limpiar la gráfica anterior
                            ax.plot(data_values, 'o-') # Graficar los nuevos puntos
                            ax.set_xlabel("Índice del Dato")
                            ax.set_ylabel("Valor del Dato")
                            ax.set_title("Datos de DMA en Tiempo Real")
                            ax.grid(True)

                            # Fijar los límites del eje Y entre 0 y 4000
                            ax.set_ylim(-2000, 2000)

                            # Ajustar los límites del eje X
                            ax.set_xlim(0, len(data_values) - 1)

                            plt.draw()
                            plt.pause(0.001) # Pequeña pausa para actualizar la gráfica

                    except ValueError:
                        print("Error al convertir datos a números. Saltando este set.")
                else:
                    print("No se encontró el inicio del set de datos ('DMA x:') en el mensaje recibido.")

        time.sleep(0.05) # Pequeña espera para no sobrecargar la CPU

except serial.SerialException as e:
    print(f"Error de puerto serial: {e}")
except KeyboardInterrupt:
    print("Programa terminado por el usuario.")
finally:
    if 'ser' in locals() and ser.isOpen():
        ser.close()
        print("Puerto serial cerrado.")
    plt.ioff()
    plt.show()
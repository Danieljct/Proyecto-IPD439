import serial
import struct
import time
import csv

# Configuración del puerto serial
SERIAL_PORT = 'COM18'
BAUD_RATE = 2000000

buffer = bytearray()
start_marker = b'DMA x:'
end_marker = b'END.'

output_file = 'output.csv'

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Puerto serial {SERIAL_PORT} abierto correctamente.")

    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)

        while True:
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
                        writer.writerow(data_values)  # Escribe una línea por paquete
                        print(f"Guardados {len(data_values)} valores")

                    except struct.error as e:
                        print(f"Error al desempaquetar: {e}")

            time.sleep(0.005)

except serial.SerialException as e:
    print(f"Error de puerto serial: {e}")
except KeyboardInterrupt:
    print("Programa terminado por el usuario.")
finally:
    if 'ser' in locals() and ser.isOpen():
        ser.close()
        print("Puerto serial cerrado.")

import os
import re

def combinar_archivos_linea_por_linea(prefijo="magnitudes", extension=".csv", archivo_salida="magnitudes_combinadas.csv"):    
    archivos_a_combinar = []
    
    # Obtener el listado de archivos en el directorio actual
    todos_los_archivos = os.listdir('.')
    
    # Filtrar y ordenar los archivos por su número
    patron = re.compile(rf'^{prefijo}(\d+){extension}$')
    for nombre_archivo in todos_los_archivos:
        match = patron.match(nombre_archivo)
        if match:
            numero = int(match.group(1))
            archivos_a_combinar.append((numero, nombre_archivo))
            
    archivos_a_combinar.sort() # Ordena por el número extraído

    if not archivos_a_combinar:
        print(f"No se encontraron archivos con el patrón '{prefijo}X{extension}' en el directorio actual.")
        return

    print(f"Archivos encontrados para combinar (en orden): {[f[1] for f in archivos_a_combinar]}")

    try:
        # Abrir el archivo de salida en modo escritura ('w')
        # Si ya existe, se sobrescribe.
        with open(archivo_salida, 'w') as outfile:
            for numero, nombre_archivo in archivos_a_combinar:
                print(f"Copiando líneas de: {nombre_archivo}")
                try:
                    # Abrir cada archivo de entrada en modo lectura ('r')
                    with open(nombre_archivo, 'r') as infile:
                        # Leer todas las líneas del archivo actual y escribirlas en el archivo de salida
                        for line in infile:
                            outfile.write(line)
                except FileNotFoundError:
                    print(f"Error: El archivo '{nombre_archivo}' no fue encontrado. Saltando.")
                except Exception as e:
                    print(f"Error al leer/escribir el archivo '{nombre_archivo}': {e}")
        
        print(f"\n¡Todas las líneas se han copiado exitosamente en '{archivo_salida}'!")
        
    except Exception as e:
        print(f"Ocurrió un error general al crear el archivo de salida: {e}")

combinar_archivos_linea_por_linea()
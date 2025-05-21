import matplotlib.pyplot as plt
import numpy as np
import csv

data = []
with open('output.csv', 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        data.append([float(val) for val in row])

if data:
    fs = 6660
    n = len(data[0])
    freqs = np.linspace(0, fs / 2, n)

    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot(freqs, data[0])
    ax.set_xlabel("Frecuencia (Hz)")
    ax.set_ylabel("Valor")
    ax.set_title("FFT guardada en CSV (Animación)")
    ax.grid(True)
    ax.set_ylim(np.min(data),30000)

    for i, row in enumerate(data):
        line.set_ydata(row)
        ax.set_title(f"FFT guardada en CSV (Frame {i+1}/{len(data)})")
        plt.pause(0.05)

    plt.ioff()
    plt.show()

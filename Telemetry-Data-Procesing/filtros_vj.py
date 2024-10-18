from scipy.fft import fft
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as R
from matplotlib.animation import FuncAnimation

## Cambios en el código ##
## Se utiliza los angulos en radianes debido a errores en la conversión de grados a radianes que lleva a errores en el giroscopio
## Se recomienda entregar los angulos desde la IMU en radianes


fs = 234 ## frecuencia de muestreo
cutoffh = 0.1 # Frecuencia de corte del filtro
order = 1  # Orden del filtr

# Función para crear un filtro pasa altas
def butter_highpass(cutoff, fs, order):
    nyq = 0.5 * fs  # Frecuencia de Nyquist
    normal_cutoff = cutoff / nyq  # Normalizar la frecuencia de corte
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a


# Inicializar los filtros
b, a = butter_highpass(cutoffh, fs, order)
kernel = np.ones(10) / 10

# Leer el archivo CSV
df = pd.read_csv('Sample_x_mov.csv', delimiter=',', header=0)
df.columns = ['i', 'gx', 'gy', 'gz', 'ax', 'ay', 'az', 'mx', 'my', 'mz']

N = len(df['i'])
x = np.arange(N)

# filtrado de los datos del giroscopio
gyr = np.column_stack((df['gx'],df['gy'], df['gz'])) * np.pi / 180
filgyr = np.zeros_like(gyr)
for j in range(gyr.shape[1]):
    filgyr[:, j] = filtfilt(b, a, gyr[:, j])

# Aceleración
acc = np.column_stack((df['ax'],df['ay'],df['az']))
acc_100 = acc[:400, :]
offset = np.sum(acc_100, axis=0) /400

# Calcular el ángulo de rotación
ang = np.zeros_like(gyr)
for i in range(1,N):
    ang[i] = ang[i-1] + filgyr[i] / fs   


# Crear matrices de rotación a partir de los ángulos de rotación
rotation_matrices = []
for angle in ang:
    # Crear una rotación usando los ángulos en radianes
    r = R.from_euler('xyz', angle)
    rotation_matrix = r.as_matrix()  # Obtener la matriz de rotación
    rotation_matrices.append(rotation_matrix)
rotation_matrices = np.array(rotation_matrices)

for i in range(rotation_matrices.shape[0]):
    rotation_matrices[i] = rotation_matrices[i] / np.linalg.norm(rotation_matrices[i], axis=1, keepdims=True)  # Normaliza filas


# Aceleración compensada
acc = acc - offset
acc = acc/17600 * 9.8

# Filtro de suavizado por convolución
smothed_acc = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), axis=0, arr=acc)
linvel = np.zeros_like(acc)


# Aceleración compensada
tcAcc = np.zeros_like(acc)
for i in range(len(acc)):
    tcAcc[i, :] = rotation_matrices[i] @ smothed_acc[i, :]
# Calcular la velocidad lineal
for i in range(1,N):
    linvel[i] = linvel[i-1] + tcAcc [i] * 1/fs
# Filtrar la velocidad lineal
filvel_high = np.zeros_like(linvel)
for i in range(3):  # Asumiendo que tienes 3 columnas de velocidad
    filvel_high[:, i] = filtfilt(b, a, linvel[:, i])
# Calcular la posición lineal
linpos = np.zeros_like(filvel_high)
for i in range(1,N):
    linpos[i] = linpos[i-1] + filvel_high[i] * 1/fs
# Filtrar la posición lineal
filpos = np.zeros_like(linpos)
for i in range(3):  # Asumiendo que tienes 3 columnas de velocidad
    filpos[:, i] = filtfilt(b, a, linpos[:, i])

# Calcular la FFT de la señal
signal = acc[:,0]
filtered_signal = tcAcc[:,0]
fft_signal = np.fft.fft(signal)
fft_filtered_signal = np.fft.fft(filtered_signal)
frequencies = np.fft.fftfreq(len(signal), 1/fs)
magnitude_signal = np.abs(fft_signal) **2
magnitude_filtered_signal = np.abs(fft_filtered_signal) **2
log_magnitude_signal = 10*np.log10(magnitude_signal + 1e-10)  # Se añade un pequeño número para evitar log(0)
log_magnitude_filtered_signal = 10*np.log10(magnitude_filtered_signal + 1e-10)  # Se añade un pequeño número para evitar log(0)


# Crear las figuras

# Figura 1: Aceleración por tiempo
plt.figure(figsize=(5, 5))
plt.plot(x, acc[:,0], label='ax', color='b')
plt.plot(x, acc[:,1], label='ay', color='g')
plt.plot(x, acc[:,2], label='az', color='r')
plt.xlabel('Tiempo [s]', fontsize=10)
plt.ylabel('Aceleración [m/s^2]', fontsize=10)
plt.title('Aceleración en el tiempo', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.tight_layout()

# Figura 2: Aceleración por tiempo
plt.figure(figsize=(5, 5))
plt.plot(x, smothed_acc[:,0], label='ax', color='b')
plt.plot(x, smothed_acc[:,1], label='ay', color='g')
plt.plot(x, smothed_acc[:,2], label='az', color='r')
plt.xlabel('Tiempo [s]', fontsize=10)
plt.ylabel('Aceleración [m/s^2]', fontsize=10)
plt.title('Aceleración suavizada en el tiempo', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.tight_layout()

# Figura 3: Aceleración por tiempo
plt.figure(figsize=(5, 5))
plt.plot(x, tcAcc[:,0], label='ax', color='b')
plt.plot(x, tcAcc[:,1], label='ay', color='g')
plt.plot(x, tcAcc[:,2], label='az', color='r')
plt.xlabel('Tiempo [s]', fontsize=10)
plt.ylabel('Aceleración [m/s^2]', fontsize=10)
plt.title('Aceleración Compensada en el tiempo', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.tight_layout()

# Figura 4: Velocidad filtrada
plt.figure(figsize=(5, 5))
plt.plot(x, filvel_high[:, 0], label='vx', color='b')
plt.plot(x, filvel_high[:, 1], label='vy', color='g')
plt.plot(x, filvel_high[:, 2], label='vz', color='r')
plt.xlabel('Tiempo [s]', fontsize=10)
plt.ylabel('Velocidad [m/s]', fontsize=10)
plt.title('Velocidad HP', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.tight_layout()

# Figura 5: Posición lineal filtrada
plt.figure(figsize=(5, 5))
plt.plot(x, filpos[:, 0], label='px', color='b')
plt.plot(x, filpos[:, 1], label='py', color='g')
plt.plot(x, filpos[:, 2], label='pz', color='r')
plt.xlabel('Tiempo [s]', fontsize=10)
plt.ylabel('Posción [m]', fontsize=10)
plt.title('Posición lineal HP', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.tight_layout()

# Figura 6: Posición lineal
plt.figure(figsize=(5, 5))
plt.plot(x, linpos[:, 0], label='px', color='b')
plt.plot(x, linpos[:, 1], label='py', color='g')
plt.plot(x, linpos[:, 2], label='pz', color='r')
plt.xlabel('Posción [m]', fontsize=10)
plt.ylabel('Amplitud', fontsize=10)
plt.title('Posición lineal', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.tight_layout()

# Figura 7: (roll, pitch, yaw)
plt.figure(figsize=(5, 5))
plt.plot(x, ang[:, 0], label='Roll', color='b')
plt.plot(x, ang[:, 1], label='Pitch', color='g')
plt.plot(x, ang[:, 2], label='Yaw', color='r')
plt.xlabel('Tiempo [s]', fontsize=10)
plt.ylabel('Angulo [rad]', fontsize=10)
plt.title('Roll, Pitch, Yaw', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.tight_layout()

# Figura 8: Giroscopio
plt.figure(figsize=(5, 5))
plt.plot(x, gyr[:, 0], label='gx', color='b')
plt.plot(x, gyr[:, 1], label='gy', color='g')
plt.plot(x, gyr[:, 2], label='gz', color='r')
plt.xlabel('Tiempo [s]', fontsize=10)
plt.ylabel('Velocidad angular [rad/s]', fontsize=10)
plt.title('Giroscopio', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.tight_layout()

# Figura 9: FFT de la señal
plt.figure(figsize=(5, 5))
plt.plot(frequencies, log_magnitude_signal, label='Ax', color='b')
plt.plot(frequencies, log_magnitude_filtered_signal, label='tcAx', color='r')
plt.xlabel('Frecuencia [Hz]', fontsize=10)
plt.ylabel('Amplitud 10log', fontsize=10)
plt.xlim(0)
plt.title('FFT de la señal', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.tight_layout()

# Mostrar todas las figuras
plt.show()

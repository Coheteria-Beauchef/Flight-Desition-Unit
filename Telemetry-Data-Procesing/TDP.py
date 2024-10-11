from scipy.fft import fft
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as R



fs = 133 # frecuencia de muestreo
cutoff = 0.3 # Frecuencia de corte del filtro
order = 1  # Orden del filtr0

def butter_highpass(cutoff, fs, order=1):
    nyq = 0.5 * fs  # Frecuencia de Nyquist
    normal_cutoff = cutoff / nyq  # Normalizar la frecuencia de corte
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

b, a = butter_highpass(cutoff, fs, order)


df = pd.read_csv('LoggedData_CalInertialAndMag.csv', delimiter=',', header=0)
df.columns = ['i', 'roll', 'pitch', 'yaw', 'ax', 'ay', 'az', 'mx', 'my', 'mz']


N = len(df['i'])
x = np.arange(N)/fs

acc = np.column_stack((df['ax'],df['ay'],df['az']))
ang = np.column_stack((df['roll'],df['pitch'], df['yaw']))

ang_rad = np.radians(ang)

tcAcc = np.zeros_like(acc)
linvel = np.zeros_like(tcAcc)
filvel = np.zeros_like(linvel)
linpos = np.zeros_like(filvel)
filpos = np.zeros_like(linpos)


rotation_matrices = []
for angle in ang_rad:
    # Crear una rotación usando los ángulos en radianes
    r = R.from_euler('xyz', angle)
    rotation_matrix = r.as_matrix()
    rotation_matrices.append(rotation_matrix)
    
rotation_matrices = np.array(rotation_matrices)
print(np.shape(rotation_matrices))

for i in range(rotation_matrices.shape[0]):
    rotation_matrices[i] = rotation_matrices[i] / np.linalg.norm(rotation_matrices[i], axis=1, keepdims=True)  # Normaliza filas
for i in range(len(acc)):
    tcAcc[i, :] = rotation_matrices[i] @ acc[i, :]
for i in range(1,N):
    linvel[i] = linvel[i-1] + tcAcc [i] * 1/fs
for i in range(3):  
    filvel[:, i] = filtfilt(b, a, linvel[:, i])
for i in range(1,N):
    linpos[i] = linpos[i-1] + filvel[i] * 1/fs
b, a = butter_highpass(2*cutoff, fs, order)
for i in range(3): 
    filpos[:, i] = filtfilt(b, a, linpos[:, i])


plt.style.use('ggplot')

plt.figure(figsize=(12, 10))

# Gráfico 1: Aceleración por tiempo
plt.subplot(3, 2, 1)
plt.plot(x, df['ax'], label='ax', color='b')
plt.plot(x, df['ay'], label='ay', color='g')
plt.plot(x, df['az'], label='az', color='r')
plt.xlabel('Tiempo [s]', fontsize=10)
plt.ylabel('Amplitud', fontsize=10)
plt.title('Aceleración por tiempo', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.5)

# Gráfico 2: Tilt acceleration
plt.subplot(3, 2, 2)
plt.plot(x, tcAcc[:, 0], label='ax', color='b')
plt.plot(x, tcAcc[:, 1], label='ay', color='g')
plt.plot(x, tcAcc[:, 2], label='az', color='r')
plt.xlabel('Tiempo [s]', fontsize=10)
plt.ylabel('Amplitud', fontsize=10)
plt.title('Tilt acceleration', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.5)

# Gráfico 3: Velocidad filtrada
plt.subplot(3, 2, 3)
plt.plot(x, filvel[:, 0], label='vx', color='b')
plt.plot(x, filvel[:, 1], label='vy', color='g')
plt.plot(x, filvel[:, 2], label='vz', color='r')
plt.xlabel('Tiempo [s]', fontsize=10)
plt.ylabel('Amplitud', fontsize=10)
plt.title('Velocidad filtrada', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.5)

# Gráfico 6: Giroscopio (roll, pitch, yaw)
plt.subplot(3, 2, 6)
plt.plot(x, ang[:, 0], label='Roll', color='b')
plt.plot(x, ang[:, 1], label='Pitch', color='g')
plt.plot(x, ang[:, 2], label='Yaw', color='r')
plt.xlabel('Tiempo [s]', fontsize=10)
plt.ylabel('Amplitud', fontsize=10)
plt.title('Giroscopio (Roll, Pitch, Yaw)', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.5)

# Gráfico 4: Posición lineal
plt.subplot(3, 2, 4)
plt.plot(x, linpos[:, 0], label='px', color='b')
plt.plot(x, linpos[:, 1], label='py', color='g')
plt.plot(x, linpos[:, 2], label='pz', color='r')
plt.xlabel('Tiempo [s]', fontsize=10)
plt.ylabel('Amplitud', fontsize=10)
plt.title('Posición lineal', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.5)

# Gráfico 5: Posición lineal filtrada
plt.subplot(3, 2, 5)
plt.plot(x, filpos[:, 0], label='px', color='b')
plt.plot(x, filpos[:, 1], label='py', color='g')
plt.plot(x, filpos[:, 2], label='pz', color='r')
plt.xlabel('Tiempo [s]', fontsize=10)
plt.ylabel('Amplitud', fontsize=10)
plt.title('Posición lineal filtrada', fontsize=12, fontweight='bold')
plt.legend(loc='best', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.5)


# Ajustar el layout y mostrar todos los gráficos al mismo tiempo
plt.tight_layout()
plt.show()
import matplotlib.pyplot as plt
import numpy as np
import math
import pathlib

current_path = str(pathlib.Path(__file__).parent)

def load_from_txt(fileName):
    t = []
    efa = []
    # Leer el archivo
    with open(fileName, 'r') as archivo:
        # Leer cada línea del archivo
        for i, linea in enumerate(archivo):
            # Ignorar la primera línea (encabezado)
            if i == 0:
                continue
            # Separar las coordenadas usando '\t' (tabulador) como delimitador
            # t, x, y, theta, v, gamma, efa
            coordenadas = linea.strip().split('\t')
                
            # Convertir las coordenadas a números y agregar a los arreglos x e y
            t.append(float(coordenadas[0]))
            efa.append(float(coordenadas[6]))

    return t, efa

t1, efa1 = load_from_txt(current_path + "/sim_results/1k_0_1.txt")
t2, efa2 = load_from_txt(current_path + "/sim_results/2k_0_5.txt")
t3, efa3 = load_from_txt(current_path + "/sim_results/3k_1_0.txt")
t4, efa4 = load_from_txt(current_path + "/sim_results/4k_2_0.txt")
t5, efa5 = load_from_txt(current_path + "/sim_results/5k_3_0.txt")
t6, efa6 = load_from_txt(current_path + "/sim_results/6k_4_0.txt")
plt.plot(t1, efa1, "-b", label="k = 0.1 [s^-1]")
plt.plot(t2, efa2, "-r", label="k = 0.5 [s^-1]")
plt.plot(t3, efa3, "-g", label="k = 1.0 [s^-1]")
plt.plot(t4, efa4, "-m", label="k = 2.0 [s^-1]")
plt.plot(t5, efa5, "-k", label="k = 3.0 [s^-1]")
plt.plot(t6, efa6, "-y", label="k = 4.0 [s^-1]")
plt.legend()
plt.xlabel("Tiempo [s]")
plt.ylabel("Error lateral [m]")
plt.grid(True)
plt.title("Controlador Stanley: v = 0.25m/s")

RMSE_array = []
efa_array = [efa1, efa2, efa3, efa4, efa5, efa6]
k_array = [0.1, 0.5, 1.0, 2.0, 3.0, 4.0]
for ei in efa_array:
    MSE = np.square(np.subtract(0, ei)).mean() 
    RMSE = math.sqrt(MSE)
    RMSE_array.append(RMSE)

plt.subplots(1)
plt.plot(k_array, RMSE_array)
plt.xlabel("k [1/s]")
plt.ylabel("RMSE [m]")
plt.grid()
plt.title("Root Mean Square Error")

plt.show()
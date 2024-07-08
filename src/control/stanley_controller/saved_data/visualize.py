import matplotlib.pyplot as plt
import numpy as np

file_name = "astarprueba4"
base_path_data = file_name + "_data.txt"
base_path_path = file_name + "_path.txt"
resolution = 0.05           # [m/cell]
rows = 37                   # Filas [celdas]
cols = 30                   # Columnas [celdas]
goal =  (0.9, 1.5)          # [m] (None for no consider it)
start = (0.28, 0.22)          # [m] (None for no consider it)
gamma_sat = 3.1416/6.0      # [rad]  (None for no consider it)

def main():
    t, x, y, theta, v, gamma, efa = load_data_from_txt(base_path_data)
    xp, yp, thetap = load_path_from_txt(base_path_path)

    plt.plot(x, y,   "-b", label="robot path")
    plt.plot(xp, yp, "-r", label="desired path")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.xlim(0, resolution*cols)
    plt.ylim(0, resolution*rows)
    if goal is not None:
        plt.scatter(goal[0], goal[1], marker='*', s=200, label="goal")
    if start is not None:
        plt.scatter(start[0], start[1], marker='o', s=200, label="start")
    plt.axis('equal')
    plt.legend()
    plt.grid()
    plt.title("Path")

    plt.subplots(1)
    plt.plot(t, theta, "-b")
    plt.xlabel("t [s]")
    plt.ylabel("theta [rad]")
    plt.grid()
    plt.title("Robot Theta")

    plt.subplots(1)
    plt.plot(thetap, "-b")
    plt.xlabel("i")
    plt.ylabel("theta [rad]")
    plt.grid()
    plt.title("Desired Theta")

    plt.subplots(1)
    plt.plot(t, gamma, "-m")
    plt.axhline(y =  gamma_sat, color = 'b', linestyle = '--', label = "gamma sat. max.") 
    plt.axhline(y = -gamma_sat, color = 'r', linestyle = '--', label = "gamma sat. min.")
    plt.xlabel("t [s]")
    plt.ylabel("gamma [rad]")
    plt.legend()
    plt.grid()
    plt.title("Robot gamma")

    plt.subplots(1)
    plt.plot(t, efa, "-b")
    plt.xlabel("t [s]")
    plt.ylabel("efa [m]]")
    plt.grid()

    MSE = np.square(np.subtract(0, efa)).mean()     # Mean Square Error
    RMSE = np.sqrt(MSE)                             # Root Mean Square Error
    plt.title(f"Cross Track Error - RMSE: {round(RMSE, 2)} [m]")

    plt.show()

def load_data_from_txt(fileName):
    t = []
    x = []
    y = []
    theta = []
    v = []
    gamma = []
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
            x.append(float(coordenadas[1]))
            y.append(float(coordenadas[2]))
            theta.append(float(coordenadas[3]))
            v.append(float(coordenadas[4]))
            gamma.append(float(coordenadas[5]))
            efa.append(float(coordenadas[6]))

    return t, x, y, theta, v, gamma, efa

def load_path_from_txt(fileName):
    x = []
    y = []
    theta = []
    # Leer el archivo
    with open(fileName, 'r') as archivo:
        # Leer cada línea del archivo
        for i, linea in enumerate(archivo):
            # Ignorar la primera línea (encabezado)
            if i == 0:
                continue
            # Separar las coordenadas usando '\t' (tabulador) como delimitador
            # t, x, y, theta
            coordenadas = linea.strip().split('\t')
                
            # Convertir las coordenadas a números y agregar a los arreglos x e y
            x.append(float(coordenadas[0]))
            y.append(float(coordenadas[1]))
            theta.append(float(coordenadas[2]))

    return x, y, theta

if __name__ == '__main__':
    main()




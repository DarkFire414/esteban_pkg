import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from queue import PriorityQueue

# Función para generar el mapa (0: libre, 1: obstáculo)
def generar_mapa():
    return np.zeros((5, 5), dtype=int)

# Función para generar el camino inicial con A*
def a_estrella(mapa, inicio, destino):
    # Implementa tu lógica del algoritmo A* aquí y retorna el camino encontrado
    # Puedes usar una estructura de datos como una lista de coordenadas [(x1, y1), (x2, y2), ...]
    camino = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]
    return camino

# Función para animar la generación y seguimiento del camino
def animar_a_estrella(mapa, inicio, destino):
    fig, ax = plt.subplots()
    ln, = plt.plot([], [], 'ro-')
    punto, = plt.plot([], [], 'bo')

    def init():
        ax.set_xlim(0, mapa.shape[0])
        ax.set_ylim(0, mapa.shape[1])
        return ln, punto

    def update(frame):
        if frame == 0:
            # Inicializa el camino con A*
            camino = a_estrella(mapa, inicio, destino)
            update.camino = camino

        if frame < len(update.camino):
            ln.set_data(*zip(*update.camino[:frame + 1]))
            punto.set_xdata(update.camino[frame][0])
            punto.set_ydata(update.camino[frame][1])

            # Aquí puedes simular una actualización del camino
            # Por ejemplo, cambiar la posición del destino después de un cierto número de frames
            if frame == 3:
                nuevo_destino = (2, 4)
                nuevo_camino = a_estrella(mapa, inicio, nuevo_destino)
                update.camino = nuevo_camino

        return ln, punto

    update.camino = []  # Almacena el camino para usarlo en la actualización

    ani = FuncAnimation(fig, update, frames=len(update.camino) + 1, init_func=init, blit=True)
    plt.show()

# Puntos de inicio y destino
inicio = (0, 0)
destino = (4, 4)

# Genera el mapa y realiza la animación
mapa = generar_mapa()
animar_a_estrella(mapa, inicio, destino)

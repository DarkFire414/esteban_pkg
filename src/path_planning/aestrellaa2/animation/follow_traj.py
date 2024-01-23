import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

def animate_trajectory(mapa, trayectoria, width, height):
    # Crear la figura y el eje
    fig, ax = plt.subplots()
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)

    # Activar la rejilla
    ax.grid(True)

    # Inicializar el punto y la línea
    point, = ax.plot([], [], marker='o', color='r')
    line, = ax.plot([], [], color='b')

    # Crear el grid como una imagen e invertir el eje y
    img = ax.imshow(np.flipud(mapa), cmap='binary', interpolation='none', extent=(0, width, 0, height))

    # Función de inicialización de la animación
    def init():
        point.set_data([], [])
        line.set_data([], [])
        return point, line, img

    # Función de actualización para la animación
    def update(frame):
        x, y = trayectoria[frame]

        # Actualizar el punto
        point.set_data([x], [y])

        # Actualizar la línea
        line.set_xdata(np.append(line.get_xdata(), x))
        line.set_ydata(np.append(line.get_ydata(), y))

        return point, line, img

    # Crear la animación
    ani = FuncAnimation(fig, update, frames=len(trayectoria), init_func=init, blit=True, interval=20)

    # Mostrar la animación
    plt.show()


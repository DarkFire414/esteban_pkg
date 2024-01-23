import numpy as np

def circularObstacle(center_x, center_y, radius, inflation_factor, grid):
    centro_x = center_x
    centro_y = center_y
    radio = radius
    inflationFact = inflation_factor
    grid = grid  # Inicializar el atributo 'grid' con el valor pasado como parámetro

    rows, cols = grid.shape  # Obtener las dimensiones del grid

    theta = np.linspace(0, 2 * np.pi, 100)
    x = np.round(centro_x + radio * np.cos(theta)).astype(int)
    y = np.round(centro_y + radio * np.sin(theta)).astype(int)

    # Asegurarse de que las coordenadas estén dentro de los límites del grid
    x = np.clip(x, 0, rows - 1)
    y = np.clip(y, 0, cols - 1)

    # Asignar el valor 1 en el grid para cada par de coordenadas
    grid[x, y] = 1
    return grid
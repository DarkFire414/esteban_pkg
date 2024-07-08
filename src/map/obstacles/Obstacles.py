import numpy as np
import math

def circularObstacle(center_x, center_y, radius, inflation_factor, grid):
    centro_x = center_x
    centro_y = center_y
    radio = radius
    inflationFact = inflation_factor
    grid = grid  # Inicializar el atributo 'grid' con el valor pasado como parámetro

    rows, cols = grid.shape  # Obtener las dimensiones del grid

    theta = np.linspace(0, 2 * np.pi, 100)
    x = np.round(centro_x + radio * np.cos(theta) * inflationFact).astype(int)
    y = np.round(centro_y + radio * np.sin(theta) * inflationFact).astype(int)

    # Asegurarse de que las coordenadas estén dentro de los límites del grid
    x = np.clip(x, 0, rows - 1)
    y = np.clip(y, 0, cols - 1)

    # Asignar el valor 1 en el grid para cada par de coordenadas
    grid[x, y] = 1
    return grid

def staticCircularObstacle(Diametro, celdasInflacion): #Devuelve una sola grid para cada obstaculo
    diametro_celdas = math.ceil((1/5)*Diametro) #Convertimos a su equivalente en celdas y le sumamos dos para el inflado         
    rows, cols = diametro_celdas+celdasInflacion*2, diametro_celdas+celdasInflacion*2
    grid = np.zeros((rows, cols), dtype=int) #Tamaño supuesto o esperado del obstaculo inflado
    for i in range(rows):
        for j in range(cols):
            grid[i, j] = 1
    if rows > 2:
        grid[0,   0] = 0
        grid[0,  -1] = 0
        grid[-1,  0] = 0
        grid[-1, -1] = 0
    return grid

def staticCircularObstacles(listaObstaculos, celdasInflacion): #Devuelve una lista dee grids para cada obstaculo
    gridsObstaculos = []
    for obstaculo in listaObstaculos:
        Diametro = obstaculo[3]
        diametro_celdas = math.ceil((1/5)*Diametro) #Convertimos a su equivalente en celdas y le sumamos dos para el inflado         
        rows, cols = diametro_celdas+celdasInflacion*2, diametro_celdas+celdasInflacion*2
        grid = np.zeros((rows, cols), dtype=int) #Tamaño supuesto o esperado del obstaculo inflado
        for i in range(rows):
            for j in range(cols):
                grid[i, j] = 1
        if rows > 2:
            grid[0,   0] = 0
            grid[0,  -1] = 0
            grid[-1,  0] = 0
            grid[-1, -1] = 0

        if celdasInflacion == 0:
            grid = np.pad(grid, pad_width=3, mode='constant', constant_values=0)

        gridsObstaculos.append(grid)
    return gridsObstaculos

def robotbox(theta0): #Añade una caja al rededor del robot para evitar rutas imposibles
    if((theta0 >= (np.pi*8/4-np.pi/8) and theta0 <= (np.pi*8/4)) or (theta0 >= 0 and theta0 <= (np.pi/4-np.pi/8))):
        caja = np.array([[1, 1, 1, 1, 1],
                         [1, 1, 0, 0, 0],
                         [1, 1, 0, 0, 0],
                         [1, 1, 0, 0, 0],
                         [1, 1, 1, 1, 1]])
        return caja
    if(theta0 > (np.pi/4-np.pi/8) and theta0 <= (np.pi/2-np.pi/8)):
        caja = np.array([[1, 1, 1, 0, 0],
                         [1, 1, 0, 0, 0],
                         [1, 1, 0, 0, 1],
                         [1, 1, 1, 1, 1],
                         [1, 1, 1, 1, 1]])
        caja = np.flipud(caja)
        return caja
    if(theta0 > (np.pi/2-np.pi/8) and theta0 <= (np.pi*3/4-np.pi/8)):
        caja = np.array([[1, 0, 0, 0, 1],
                         [1, 0, 0, 0, 1],
                         [1, 0, 0, 0, 1],
                         [1, 1, 1, 1, 1],
                         [1, 1, 1, 1, 1]])
        caja = np.flipud(caja)
        return caja
    if(theta0 > (np.pi*3/4-np.pi/8) and theta0 <= (np.pi-np.pi/8)):
        caja = np.array([[0, 0, 1, 1, 1],
                         [0, 0, 0, 1, 1],
                         [1, 0, 0, 1, 1],
                         [1, 1, 1, 1, 1],
                         [1, 1, 1, 1, 1]])
        caja = np.flipud(caja)
        return caja
    if(theta0 > (np.pi-np.pi/8) and theta0 <= (np.pi*5/4-np.pi/8)):
        caja = np.array([[1, 1, 1, 1, 1],
                         [0, 0, 0, 1, 1],
                         [0, 0, 0, 1, 1],
                         [0, 0, 0, 1, 1],
                         [1, 1, 1, 1, 1]])
        return caja
    if(theta0 > (np.pi*5/4-np.pi/8) and theta0 <= (np.pi*6/4-np.pi/8)):
        caja = np.array([[1, 1, 1, 1, 1],
                         [1, 1, 1, 1, 1],
                         [1, 0, 0, 1, 1],
                         [0, 0, 0, 1, 1],
                         [0, 0, 1, 1, 1]])
        caja = np.flipud(caja)
        return caja
    if(theta0 > (np.pi*6/4-np.pi/8) and theta0 <= (np.pi*7/4-np.pi/8)):
        caja = np.array([[1, 1, 1, 1, 1],
                         [1, 1, 1, 1, 1],
                         [1, 0, 0, 0, 1],
                         [1, 0, 0, 0, 1],
                         [1, 0, 0, 0, 1]])
        caja = np.flipud(caja)
        return caja
    if(theta0 > (np.pi*7/4-np.pi/8) and theta0 <= (np.pi*8/4-np.pi/8)):
        caja = np.array([[1, 1, 1, 1, 1],
                         [1, 1, 1, 1, 1],
                         [1, 1, 0, 0, 1],
                         [1, 1, 0, 0, 0],
                         [1, 1, 1, 0, 0]])
        caja = np.flipud(caja)
        return caja

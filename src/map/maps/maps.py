from obstacles.Obstacles import circularObstacle

def maps(grid, option):
    rows, cols = grid.shape
    if option == 1: #Laberinto
        for i in range(rows):
            for j in range(cols):
                if (i < 1 and 4 < j) or i > 38 or j > 38 or j < 1: #Paredes
                    grid[i, j] = 1
                if 25 < i < 40 and 10 < j < 12: #Cuarto No. 1 [vertical]
                    grid[i, j] = 1
                if 25 < i < 27 and j < 5: #linea horizontal
                    grid[i, j] = 1
                if 0 <= i < 15 and 28 < j < 30: #Cuarto No. 2 [vertical]
                    grid[i, j] = 1
                if 13 < i < 15 and 26 > j > 4: #linea horizontal
                    grid[i, j] = 1
                if 0 <= i < 10 and 4 < j < 6: #linea vertical
                    grid[i, j] = 1
                if 5 <= i < 15 and 15 < j < 17: #Cuarto No. 3 [vertical]
                    grid[i, j] = 1
                if (26 <= i < 32 or 36 <= i < 40) and 25 < j < 27: #Cuarto No. 4 [vertical]
                    grid[i, j] = 1
                if 25 < i < 27 and (10 < j < 16 or 18 < j < 27):  # linea horizontal
                    grid[i, j] = 1
                if 25 < i < 27 and 26 < j < 37:  # linea horizontal
                    grid[i, j] = 1
                if (13 < i < 18 or 20 < i < 27) and 12 < j < 14:  # linea vertical
                    grid[i, j] = 1
                if 13 < i < 15 and 31 < j < 40:  # linea horizontal
                    grid[i, j] = 1
        return grid
    if option == 2: #Manzanas
        for i in range(rows):
            for j in range(cols):
                if (i < 1 and 6 < j) or i > (rows-2) or j > (cols-2) or j < 1: #Paredes
                    grid[i, j] = 1
        grid = circularObstacle(8, 10, 1, 1, grid) #Fia inferior
        grid = circularObstacle(8, 20, 1, 1, grid)
        grid = circularObstacle(18, 10, 1, 1, grid) #Fila media
        grid = circularObstacle(18, 20, 1, 1, grid)
        grid = circularObstacle(28, 10, 1, 1, grid) #Fila superior
        grid = circularObstacle(28, 20, 1, 1, grid)
        return grid

    if option == 3: #Contorno estático
        for i in range(rows):
            for j in range(cols):
                if (i < 1 and 6 < j) or i > (rows-2) or j > (cols-2) or j < 1: #Paredes
                    grid[i, j] = 1
        return grid

def add_obstacle_to_map(grid, obstacle, X, Y, Diametro):
    rows, cols = obstacle.shape
    celdax = round((1/5) * X) - round((1/5)*(Diametro/2))  #celda en x de la esquina superior izquierda donde comienza el obstaculo dentro del mapa
    celday = round((1/5) * Y) - round((1/5)*(Diametro/2))  #celda en x de la esquina superior izquierda donde comienza el obstaculo dentro del mapa
    for i in range(rows):
        for j in range(cols):
            if (celdax + i) >= 0 and (celday+j) >= 0: #Unicamente las coordenadas que esten dentro del grid se meten
                grid[celdax + i][celday+j] = obstacle[i][j]
    return grid

def add_obstacles_to_map(grid, obstacles, listaObstaculos): #Añade una lista de obstaculos al mapa
    contador = 0
    for obstacle in obstacles:
        rows, cols = obstacle.shape
        X, Y, Diametro = listaObstaculos[contador][1], listaObstaculos[contador][2], listaObstaculos[contador][3]
        celdax = round((1/5) * X) - round((1/5)*(Diametro/2)) - 2 #celda en x de la esquina superior izquierda donde comienza el obstaculo dentro del mapa
        celday = round((1/5) * Y) - round((1/5)*(Diametro/2)) - 2 #celda en x de la esquina superior izquierda donde comienza el obstaculo dentro del mapa
        #print("x: ", celdax * 5)
        #print("y: ", celday * 5)
        contador = contador + 1
        for i in range(rows):
            for j in range(cols):
                if (celdax + i) >= 0 and (celday+j) >= 0: #Unicamente las coordenadas que esten dentro del grid se meten
                    grid[celdax + i][celday+j] = obstacle[i][j]
    return grid

def add_robotbox_to_map(grid, robotbox, x0, y0): #Añade una caja al rededor del robot para evitar rutas imposibles
    rows, cols = 5, 5
    celday = round(x0) - 2 #celda en x de la esquina superior izquierda donde comienza el obstaculo dentro del mapa
    celdax = round(y0) - 2 #celda en x de la esquina superior izquierda donde comienza el obstaculo dentro del mapa
    for i in range(rows):
        for j in range(cols):
            if (celdax + i) >= 0 and (celday+j) >= 0: #Unicamente las coordenadas que esten dentro del grid se meten
                grid[celdax + i][celday+j] = robotbox[i][j]
    return grid

'''if (2 < i < 7 and 5 < j < 10) or (17 < i < 22 and 5 < j < 10) or (32 < i < 37 and 5 < j < 10) or (2 < i < 7 and 17 < j < 22) or (17 < i < 22 and 17 < j < 22) or (32 < i < 37 and 17 < j < 22) or (2 < i < 7 and 30 < j < 35) or (17 < i < 22 and 30 < j < 35) or (32 < i < 37 and 30 < j < 35):
    grid[i,j] = 1'''

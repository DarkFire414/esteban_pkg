import numpy as np
class Global_APF:
    def __init__(self, start, goal, grid):
        self.start = start
        self.goal = goal
        self.grid = grid
    def goal(self, s, r, x, y, coords):
        X, Y = np.meshgrid(x, y)  # Coordenadas de cada nodo en la malla
        delx = np.zeros_like(X)
        dely = np.zeros_like(Y)
        for i in range(len(x)):
            for j in range(len(y)):
                d = np.sqrt(X[i][j] ** 2 + Y[i][j] ** 2)
                theta = np.arctan2(Y[i][j], X[i][j])
                if d < 2:
                    delx[i][j] = 0
                    dely[i][j] = 0
                elif d > r + s:
                    delx[i][j] = -50 * s * np.cos(theta)
                    dely[i][j] = -50 * s * np.sin(theta)
                else:
                    delx[i][j] = -50 * (d - r) * np.cos(theta)
                    dely[i][j] = -50 * (d - r) * np.sin(theta)
#!/usr/bin/env python3

from collections import deque
import numpy as np
from suaviza.mientras_llueve import approximate_b_spline_path

# Parameters
KP = 4.0  # attractive potential gain
ETA = 50.0  # repulsive potential gain
AREA_WIDTH = 40.0 # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 6

class APF_Ackermann:
    def __init__(self, start, goal, grid_size, robot_radius, grid, theta0):
        self.sx = start[0]
        self.sy = start[1]
        self.gx = goal[0]
        self.gy = goal[1]
        self.theta0 = theta0 #Define que direccion inicial debe tomar el algoritmo
        self.reso = grid_size
        self.rr   = robot_radius
        self.grid = grid #mapa
        self.coords_with_1 = [] #Lista de los obstaculos del mapa

    def calc_potential_field(self):
        minx = 0
        miny = 0
        maxx = AREA_WIDTH
        maxy = AREA_WIDTH
        xw = int(round((maxx - minx) / self.reso))
        yw = int(round((maxy - miny) / self.reso))

        # Calc attractive potential
        potential_map = np.zeros((xw, yw))
        for ix in range(xw):
            x = ix * self.reso + minx
            for iy in range(yw):
                y = iy * self.reso + miny
                potential_map[ix][iy] = 0.5 * KP * np.hypot(x - self.gx, y - self.gy)

        # Calc repulsive potential only for cells near obstacles
        for (wx, wy) in self.coords_with_1:
            wx_index = int((wx - minx) / self.reso)
            wy_index = int((wy - miny) / self.reso)
            for ix in range(max(0, wx_index - self.rr), min(xw, wx_index + self.rr + 1)):
                for iy in range(max(0, wy_index - self.rr), min(yw, wy_index + self.rr + 1)):
                    x = ix * self.reso + minx
                    y = iy * self.reso + miny
                    dw = np.hypot(x - wx, y - wy)
                    if dw <= self.rr:
                        if dw <= 0.1:
                            dw = 0.1
                        potential_map[ix][iy] += 0.5 * ETA * (1.0 / dw - 1.0 / self.rr) ** 2

        return potential_map, minx, miny

    def calc_attractive_potential(self, x, y):
        return 0.5 * KP * np.hypot(x - self.gx, y - self.gy)

    def get_motion_model(self):
        # dx, dy
        motion = [[1, 0],
                [0, 1],
                [-1, 0],
                [0, -1],
                [-1, -1],
                [-1, 1],
                [1, -1],
                [1, 1]]

        return motion


    def oscillations_detection(self, previous_ids, ix, iy):
        previous_ids.append((ix, iy))

        if len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH:
            previous_ids.popleft()

        # check if contains any duplicates by copying into a set
        previous_ids_set = set()
        for index in previous_ids:
            if index in previous_ids_set:
                return True
            else:
                previous_ids_set.add(index)
        return False

    def obstacles_coordinates(self):
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
            #Verificar si el valor en la posición actual es igual a 1
                if self.grid[i][j] != 0:
                    # Si es así, guardar las coordenadas (i, j) en la lista
                    self.coords_with_1.append((i, j))
    
    def initial_points_path(self, points):
        #sxx = hip*sen(theta), Donde hip = raiz(2)
        #syy = hip*cos(theta)
        sxx = [self.sx]
        syy = [self.sy]
        for i in range(points):
            sxx.append(self.sx + round(np.sqrt(2)*i*np.sin(self.theta0)))
            syy.append(self.sy + round(np.sqrt(2)*i*np.cos(self.theta0)))
        return sxx, syy

    def smooth_path(self, x, y):
        if x:
            n_course_point = 400
            rax, ray, heading, curvature = approximate_b_spline_path(
                y, x, n_course_point, s=0.5)
            return list(zip(ray, rax, heading))
        else:
            return False

    def potential_field_planning(self):
        # calc potential field
        self.obstacles_coordinates() #Calcula las coordenadas de los obstaculos en el mapa para calcular el campo        
        pmap, minx, miny = self.calc_potential_field()

        #Definimos las coordenadas iniciales
        puntos_iniciales = 5
        rx, ry = self.initial_points_path(puntos_iniciales) #Modificamos el punto de inicio generando coordenadas en linea recta para posicionar la ruta siempre de frente al robot

        # search path
        d = np.hypot(rx[-1] - self.gx, ry[-1] - self.gy)
        ix = round((rx[-1] - minx) / self.reso)
        iy = round((ry[-1] - miny) / self.reso)
        gix = round((self.gx - minx) / self.reso)
        giy = round((self.gy - miny) / self.reso)


        motion = self.get_motion_model()
        previous_ids = deque()

        while d >= self.reso:
            minp = float("inf")
            minix, miniy = -1, -1
            for i, _ in enumerate(motion):
                inx = int(ix + motion[i][0])
                iny = int(iy + motion[i][1])
                if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                    p = float("inf")  # outside area
                    print("outside potential!")
                else:
                    p = pmap[inx][iny]
                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny
            ix = minix
            iy = miniy
            xp = ix * self.reso + minx
            yp = iy * self.reso + miny
            d = np.hypot(self.gx - xp, self.gy - yp)
            rx.append(xp)
            ry.append(yp)

            if (self.oscillations_detection(previous_ids, ix, iy)):
                print("Oscillation detected at ({},{})!".format(ix, iy))
                break

        print("Goal!!")

        #Devuelve raw path y smooth path 

        return self.smooth_path(rx, ry)

#!/usr/bin/env python3

from collections import deque
import numpy as np
from suaviza.mientras_llueve import approximate_b_spline_path

# Parameters
KP = 4.0  # attractive potential gain
ETA = 500.0  # repulsive potential gain
AREA_WIDTH = 40.0 # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3

class APF_Ackermann:
    def __init__(self, start, goal, ox, oy, grid_size, robot_radius, grid):
        self.sx = start[0]
        self.sy = start[1]
        self.gx = goal[0]
        self.gy = goal[1]
        self.ox = ox
        self.oy = oy
        self.reso = grid_size
        self.rr   = robot_radius
        self.grid = grid
        self.rows, self.cols = self.grid.shape
        self.wall_coordinates = []
        self.wall_distance = 2
        # Coordenadas de las paredes del cuadrado de 40x40
        for x in range(0, 30):  # Línea superior e inferior
            self.wall_coordinates.append((x, 1))
            self.wall_coordinates.append((x, 29))
        for y in range(1, 37):  # Lados izquierdo y derecho
            self.wall_coordinates.append((1, y))
            self.wall_coordinates.append((36, y))

    def calc_potential_field(self):
        minx = 0
        miny = 0
        maxx = AREA_WIDTH
        maxy = AREA_WIDTH
        xw = int(round((maxx - minx) / self.reso))
        yw = int(round((maxy - miny) / self.reso))

        # calc each potential
        pmap = [[0.0 for _ in range(yw)] for _ in range(xw)]

        for ix in range(xw):
            x = ix * self.reso + minx

            for iy in range(yw):
                y = iy * self.reso + miny
                ug = self.calc_attractive_potential(x, y)
                uo = self.calc_repulsive_potential1(x, y)
                uf = ug + uo
                pmap[ix][iy] = uf

        return pmap, minx, miny


    def calc_attractive_potential(self, x, y):
        return 0.5 * KP * np.hypot(x - self.gx, y - self.gy)


    '''def calc_repulsive_potential(self, x, y):

        # search nearest obstacle
        minid = -1
        dmin = float("inf")
        for i, _ in enumerate(self.ox):
            d = np.hypot(x - self.ox[i], y - self.oy[i])
            if dmin >= d:
                dmin = d
                minid = i

        # calc repulsive potential
        dq = np.hypot(x - self.ox[minid], y - self.oy[minid])

        if dq <= self.rr:
            if dq <= 0.1:
                dq = 0.1
            repulsive_potential_obstacle = 0.5 * ETA * (1.0 / dq - 1.0 / self.rr) ** 2
        else:
            repulsive_potential_obstacle = 0.0

        # calc repulsive potential for walls
        repulsive_potential_wall = 0.0
        for wx, wy in self.wall_coordinates:
            dw = np.hypot(x - wx, y - wy)
            if dw <= self.wall_distance:
                if dw <= 0.1:
                    dw = 0.1
                repulsive_potential_wall += 0.5 * ETA * (1.0 / dw - 1.0 / self.wall_distance) ** 2
        return repulsive_potential_obstacle + repulsive_potential_wall
    '''
    def calc_repulsive_potential1(self, x, y):
    # calc repulsive potential for walls
    repulsive_potential = 0.0
    for wx, wy in coords_with_1:
        dw = np.hypot(x - wx, y - wy)
        if dw <= self.rr:
            if dw <= 0.1:
                dw = 0.1
            repulsive_potential += 0.5 * ETA * (1.0 / dw - 1.0 / self.rr) ** 2
    return repulsive_potential

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
        pmap, minx, miny = self.calc_potential_field()

        # search path
        d = np.hypot(self.sx - self.gx, self.sy - self.gy)
        ix = round((self.sx - minx) / self.reso)
        iy = round((self.sy - miny) / self.reso)
        gix = round((self.gx - minx) / self.reso)
        giy = round((self.gy - miny) / self.reso)

        rx, ry = [self.sx], [self.sy]
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

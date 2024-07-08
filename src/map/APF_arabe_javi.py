from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from maps.maps import maps
# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 40.0 # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3

show_animation = True


def calc_potential_field(gx, gy, reso, sx, sy,rr):
    minx = 0
    miny = 0
    maxx = AREA_WIDTH
    maxy = AREA_WIDTH
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for _ in range(yw)] for _ in range(xw)]
    for ix in range(xw):
        x = ix * reso + minx
        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential1(x, y,rr)
            uf = ug + uo
            pmap[ix][iy] = uf


    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)

# esta lo hace celda a celda
"""
def calc_repulsive_potential(x, y, ox, oy,rr_list):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i
            rr = rr_list[minid]

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1
        repulsive_potential_obstacle = 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        repulsive_potential_obstacle = 0.0
    # calc repulsive potential for walls
    repulsive_potential_wall = 0.0
    for wx, wy in wall_coordinates:
        dw = np.hypot(x - wx, y - wy)
        if dw <= wall_distance:
            if dw <= 0.1:
                dw = 0.1
            repulsive_potential_wall += 0.5 * ETA * (1.0 / dw - 1.0 / wall_distance) ** 2
    return repulsive_potential_obstacle + repulsive_potential_wall
"""
 #esta agarratodo el mapa
def calc_repulsive_potential1(x, y,rr):
    # calc repulsive potential for walls
    repulsive_potential = 0.0
    for wx, wy in coords_with_1:
        dw = np.hypot(x - wx, y - wy)
        if dw <= rr:
            if dw <= 0.1:
                dw = 0.1
            repulsive_potential += 0.5 * ETA * (1.0 / dw - 1.0 / rr) ** 2
    return repulsive_potential
def get_motion_model():
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


def oscillations_detection(previous_ids, ix, iy):
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


def potential_field_planning(sx, sy, gx, gy, reso,rr):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, reso, sx, sy,rr)

    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)

    if show_animation:
        draw_heatmap(pmap)
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(ix, iy, "*k")
        plt.plot(gix, giy, "*m")

    rx, ry = [sx], [sy]
    motion = get_motion_model()
    previous_ids = deque()

    while d >= reso:
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
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)

        if (oscillations_detection(previous_ids, ix, iy)):
            print("Oscillation detected at ({},{})!".format(ix, iy))
            break

        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.01)

    print("Goal!!")

    return rx, ry


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data,vmin=0, vmax=100.0, cmap=plt.cm.Blues)


def main():
    print("potential_field_planning start")

    sx = 25.0  # start x position [m]
    sy = 15.0  # start y positon [m]
    gx = 15.0  # goal x position [m]
    gy = 25.0  # goal y position [m]
    grid_size = 1.0# potential grid size [m]
    rr = 2

    if show_animation:
        plt.grid(True)
        plt.axis("equal")
        plt.xlim(0, AREA_WIDTH)
        plt.ylim(0, AREA_WIDTH)

    # path generation
    _, _ = potential_field_planning(
        sx, sy, gx, gy, grid_size,rr)

    if show_animation:
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    # Definir las dimensiones de la cuadrícula
    rows = 40
    cols = 40
    # Generar la cuadrícula inicial (aquí puedes inicializar tu propia cuadrícula si no la tienes)
    grid1 = np.zeros((rows, cols))
    grid = maps(grid1,2)

    print(grid)

    plt.imshow(grid, cmap='binary')
    plt.gca().invert_yaxis()  # Invertir el eje y para que el origen esté en la esquina inferior izquierda
    plt.show()

    coords_with_1=[]

        # Iterar sobre todas las filas y columnas de la matriz
    for i in range(len(grid)):
        for j in range(len(grid[0])):
        #Verificar si el valor en la posición actual es igual a 1
            if grid[i][j] == 1:
                # Si es así, guardar las coordenadas (i, j) en la lista
                coords_with_1.append((i, j))
    print(coords_with_1)
    main()
    print(__file__ + " Done!!")

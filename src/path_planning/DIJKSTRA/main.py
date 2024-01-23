import numpy as np
import matplotlib.pyplot as plt
from HybridDijkstra import Dijkstra
from maps.maps import maps
from inflateMap.inflate import inflate_map

#----------------------Path tracking---------------------------------
import sys
import pathlib
import math
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import anim.draw as draw
from path_tracking.stanley_controller.sim_params import C
from path_tracking.stanley_controller.ackermann_model import Ackermann_robot, States
from path_tracking.stanley_controller.control import Control
from path_tracking.stanley_controller.trajectory import Trajectory 
#--------------------------------------------------------------------

rows = 40
cols = 40
start = (0, 3)
goal = (35, 35)
grid = np.zeros((rows, cols), dtype=int) # El mapa sera una cuadricula donde 0 es libre y 1 es ocupado
grid = maps(grid, 2)
inflated_grid = inflate_map(grid, 2)
try:
    # Aplicacion del algoritmo global que da la primer instancia de la trayectoria a seguir
    GlobalPlannerDijkstra = Dijkstra.GlobalDijkstraAckermann(start, goal, inflated_grid)
    GlobalRawPath, GlobalPath = GlobalPlannerDijkstra.dijkstra()
    GlobalRawY, GlobalRawX = zip(*GlobalRawPath)
    GlobalY, GlobalX = zip(*GlobalPath)

    #-----------------Path tracking-----------------------------------
    showAnimation = True
    cell_size = 0.05    # [m]
    # Escalar las coordenadas a metros
    GlobalX_metros = np.array(GlobalX) * cell_size
    GlobalY_metros = np.array(GlobalY) * cell_size
    # Binarizar mapa de obstáculos
    cmap = plt.get_cmap('binary')

    # Trayectoria a seguir
    traj = Trajectory()
    traj.generatePath(GlobalX_metros, GlobalY_metros)
    # Perfil de velocidad
    pathv = 0.25    # [m/s]
    T = 100.0       # Tiempo máximo de simulación
    # Condiciones iniciales
    ack = Ackermann_robot(x0=start[1]*cell_size, y0=start[0]*cell_size, theta0=np.pi/2, v0=0.0, C=C)
    time = 0.0

    lastIndex = len(GlobalX_metros)-1
    states = States()
    states.addState(time, ack, 0.0)
    control = Control(C)
    target_ind, _ = control.calc_target_index(ack, traj.px, traj.py)

    theta_old = 0.0

    while T>= time and lastIndex > target_ind:
        acc_i = control.pid_control(pathv, ack.v, C.dt)
        gamma_i, target_ind, efa = control.stanley_control(ack, traj.px, traj.py, traj.ptheta, target_ind)

        ack.update(acc_i, gamma_i)

        time += C.dt

        states.addState(time, ack, efa)

        dy = (ack.theta - theta_old) / (ack.v * C.dt)
        steer = draw.pi_2_pi(-math.atan(C.WB * dy))
        theta_old = ack.theta

        if showAnimation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            
            plt.imshow(grid, cmap=cmap, extent=[0, 2, 0, 2], origin='lower')
            plt.plot(traj.px, traj.py, ".r", label="Trayectoria")
            plt.plot(states.x, states.y, "-b", label="Seguimiento")
            plt.plot(traj.px[target_ind], traj.py[target_ind], "xg", label="target")
            
            draw.draw_car(ack.x, ack.y, ack.theta, steer, C)

            plt.legend()
            plt.axis("equal")
            plt.title("Speed[m/s]:" + str(ack.v)[:4])
            plt.pause(0.001)

    plt.subplots(1)
    # Graficar mapa de osbtáculos
    plt.imshow(grid, cmap=cmap, extent=[0, 2, 0, 2], origin='lower')
  
    plt.plot(traj.px, traj.py, '-r', label="Trayectoria")
    plt.plot(states.x, states.y, '-b', label="Seguimiento")

    plt.legend()
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Seguimiento de trayectoria Dijkstra (inicio = ('+str(start[1]*cell_size)[:4]+', '+str(start[0]*cell_size)[:4]+'))'+'(final = ('+str(goal[1]*cell_size)[:4]+', '+str(goal[0]*cell_size)[:4]+'))')
    plt.text(start[0]*0.05, start[1]*0.05, 'Inicio', color='red', fontsize=12)
    plt.text(goal[0]*0.05, goal[1]*0.05, 'Fin', color='red', fontsize=12)
    plt.show()
    #-----------------------------------------------------------------

    """
    print(GlobalX)
    print(GlobalY)
    #MUESTRA DEL MAPA INFLADO
    plt.subplot(2, 2, 1)
    cmap = plt.get_cmap('binary')
    plt.imshow(inflated_grid, cmap=cmap)
    #plt.plot(GlobalRawX, GlobalRawY, '-b', label="Approximated B-Spline path")
    plt.plot(GlobalX, GlobalY, '-r', label="Approximated B-Spline path")
    plt.gca().invert_yaxis()
    plt.xlabel('Eje X')
    plt.ylabel('Eje Y')
    plt.title('Mapa inflado con el camino generado Dijkstra (inicio = ('+str(start[1])+', '+str(start[0])+'))'+'(final = ('+str(goal[1])+', '+str(goal[0])+'))')
    plt.text(start[0], start[1], 'Inicio', color='red', fontsize=12)
    plt.text(goal[0], goal[1], 'Fin', color='red', fontsize=12)
    #MUESTRA DEL MAPA ORIGINAL
    plt.subplot(2, 2, 2)
    cmap = plt.get_cmap('binary')
    plt.imshow(grid, cmap=cmap)
    #plt.plot(GlobalRawX, GlobalRawY, '-b', label="Approximated B-Spline path")
    plt.plot(GlobalX, GlobalY, '-r', label="Approximated B-Spline path")
    plt.gca().invert_yaxis()
    plt.xlabel('Eje X')
    plt.ylabel('Eje Y')
    plt.title('Mapa original con el camino generado Dijkstra (inicio = ('+str(start[1])+', '+str(start[0])+'))'+'(final = ('+str(goal[1])+', '+str(goal[0])+'))')
    plt.text(start[0], start[1], 'Inicio', color='red', fontsize=12)
    plt.text(goal[0], goal[1], 'Fin', color='red', fontsize=12)
    #MUESTRA EL MAPA ORIGINAL CON EL CAMINO ORIGINAL
    plt.subplot(2, 2, 3)
    cmap = plt.get_cmap('binary')
    plt.imshow(grid, cmap=cmap)
    #plt.plot(GlobalRawX, GlobalRawY, '-b', label="Approximated B-Spline path")
    plt.plot(GlobalRawX, GlobalRawY, '-r', label="Approximated B-Spline path")
    plt.gca().invert_yaxis()
    plt.xlabel('Eje X')
    plt.ylabel('Eje Y')
    plt.title('Mapa original con el camino generado original Dijkstra (inicio = ('+str(start[1])+', '+str(start[0])+'))'+'(final = ('+str(goal[1])+', '+str(goal[0])+'))')
    plt.text(start[0], start[1], 'Inicio', color='red', fontsize=12)
    plt.text(goal[0], goal[1], 'Fin', color='red', fontsize=12)
    plt.tight_layout()
    plt.show()

    #GRAFICA PARA REPORTAR
    #MUESTRA DEL MAPA INFLADO
    plt.subplot(1, 2, 1)
    cmap = plt.get_cmap('binary')
    plt.imshow(inflated_grid, cmap=cmap)
    #plt.plot(GlobalRawX, GlobalRawY, '-b', label="Approximated B-Spline path")
    plt.plot(GlobalX, GlobalY, '-r', label="Approximated B-Spline path")
    plt.gca().invert_yaxis()
    plt.xlabel('Eje X')
    plt.ylabel('Eje Y')
    plt.title('Mapa inflado con el camino generado Dijkstra (inicio = ('+str(start[1])+', '+str(start[0])+'))'+'(final = ('+str(goal[1])+', '+str(goal[0])+'))')
    plt.text(start[0], start[1], 'Inicio', color='red', fontsize=12)
    plt.text(goal[0], goal[1], 'Fin', color='red', fontsize=12)
    #MUESTRA DEL MAPA ORIGINAL
    plt.subplot(1, 2, 2)
    cmap = plt.get_cmap('binary')
    plt.imshow(grid, cmap=cmap)
    #plt.plot(GlobalRawX, GlobalRawY, '-b', label="Approximated B-Spline path")
    plt.plot(GlobalX, GlobalY, '-r', label="Approximated B-Spline path")
    plt.gca().invert_yaxis()
    plt.xlabel('Eje X')
    plt.ylabel('Eje Y')
    plt.title('Mapa original con el camino generado Dijkstra (inicio = ('+str(start[1])+', '+str(start[0])+'))'+'(final = ('+str(goal[1])+', '+str(goal[0])+'))')
    plt.text(start[0], start[1], 'Inicio', color='red', fontsize=12)
    plt.text(goal[0], goal[1], 'Fin', color='red', fontsize=12)
    plt.tight_layout()
    plt.show()

    plt.subplot(1, 2, 1)
    cmap = plt.get_cmap('binary')
    plt.imshow(grid, cmap=cmap)
    plt.gca().invert_yaxis()
    plt.title('Mapa original')
    plt.subplot(1, 2, 2)
    cmap = plt.get_cmap('binary')
    plt.imshow(inflated_grid, cmap=cmap)
    plt.gca().invert_yaxis()
    plt.title('Mapa inflado')
    plt.tight_layout()
    plt.show()

    plt.subplot(1, 2, 1)
    cmap = plt.get_cmap('binary')
    plt.imshow(grid, cmap=cmap)
    plt.plot(GlobalRawX, GlobalRawY, '-r', label="Approximated B-Spline path")
    plt.gca().invert_yaxis()
    plt.xlabel('Eje X')
    plt.ylabel('Eje Y')
    plt.title('Dijkstra original y mapa normal(inicio = (' + str(start[1]) + ', ' + str(start[0]) + '))' + '(final = (' + str(goal[1]) + ', ' + str(goal[0]) + '))')
    plt.text(start[0], start[1], 'Inicio', color='red', fontsize=12)
    plt.text(goal[0], goal[1], 'Fin', color='red', fontsize=12)

    plt.subplot(1, 2, 2)
    cmap = plt.get_cmap('binary')
    plt.imshow(inflated_grid, cmap=cmap)
    plt.plot(GlobalRawX, GlobalRawY, '-r', label="Approximated B-Spline path")
    plt.gca().invert_yaxis()
    plt.xlabel('Eje X')
    plt.ylabel('Eje Y')
    plt.title('Dijkstra original y mapa inflado (inicio = (' + str(start[1]) + ', ' + str(start[0]) + '))' + '(final = (' + str(goal[1]) + ', ' + str(goal[0]) + '))')
    plt.text(start[0], start[1], 'Inicio', color='red', fontsize=12)
    plt.text(goal[0], goal[1], 'Fin', color='red', fontsize=12)
    plt.tight_layout()
    plt.show()
    """
except:
    print("No se encontró un camino válido.")

'''Codigo para 200 celdas
rows = 200
cols = 200
start = (10, 10)
goal = (190, 190)
# El mapa sera una cuadricula donde 0 es libre y 1 es ocupado
grid = np.zeros((rows, cols), dtype=int)
for i in range(rows):
    for j in range(cols):
        if (i < 5 and 20 < j) or i > 195 or j > 195 or j < 5:  # Paredes
            grid[i, j] = 1
        if 125 < i < 200 and 50 < j < 60:  # Cuarto No. 1 [vertical]
            grid[i, j] = 1
        if 125 < i < 130 and j < 25:  # línea horizontal
            grid[i, j] = 1
        if 0 <= i < 75 and 140 < j < 150:  # Cuarto No. 2 [vertical]
            grid[i, j] = 1
        if 68 < i < 75 and 130 > j > 20:  # línea horizontal
            grid[i, j] = 1
        if 0 <= i < 50 and 20 < j < 30:  # línea vertical
            grid[i, j] = 1
        if 25 <= i < 75 and 75 < j < 85:  # Cuarto No. 3 [vertical]
            grid[i, j] = 1
        if (130 <= i < 160 or 180 <= i < 200) and 125 < j < 135:  # Cuarto No. 4 [vertical]
            grid[i, j] = 1
        if 125 < i < 130 and (50 < j < 80 or 90 < j < 135):  # línea horizontal
            grid[i, j] = 1
        if 125 < i < 130 and 130 < j < 185:  # línea horizontal
            grid[i, j] = 1
        if (68 < i < 90 or 100 < i < 125) and 60 < j < 70:  # línea vertical
            grid[i, j] = 1
        if 68 < i < 75 and 155 < j < 200:  # línea horizontal
            grid[i, j] = 1'''

'''def save_to_txt(file_name, x, y):
    with open(file_name, 'w') as file:
        # Escribir encabezado
        file.write("\tx\ty\n")

        # Escribir datos
        for i in range(len(x)):
            xi = x[i]
            yi = y[i]

            # Escribir una línea de datos en el archivo
            file.write("{:.4f}\t{:.4f}\t".format(xi, yi))'''
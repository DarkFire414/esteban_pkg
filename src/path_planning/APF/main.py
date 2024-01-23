import matplotlib.pyplot as plt
import numpy as np
from APF.APF2 import add_goal, add_obstacle, plot_graph
from maps.maps import maps

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

# Parametros para el algoritmo
cols, rows = 40, 40
x = np.arange(-0, cols+5, 1)
y = np.arange(-0, rows+5, 1)
so = 5
ro = 2
sg = 5
rg = 1
X, Y = np.meshgrid(x, y)
# Puntos de inicio y de final
start = np.array([[0, 0]])
goal = [39, 39]
# Grafico del mapa vectorial resultante
fig, ax = plt.subplots(figsize=(10, 10))
delx, dely = add_goal(X, Y, sg, rg, goal, x, y)
plot_graph(X, Y, delx, dely, 'Goal', fig, ax, goal, 2, 0, 'b')
#Agreamos todos los obstaculos del mapa
for i in range(10, 31, 10):
    for j in range(10, 31, 10):
        delx, dely, loc, r = add_obstacle(X, Y, so, ro, delx, dely, goal, x, y, (i,j))
        plot_graph(X, Y, delx, dely, 'Obstacle', fig, ax, loc, 2, 0, 'm')

stream = ax.streamplot(X, Y, delx, dely, start_points=start, linewidth=4, cmap='autu')
#print(stream)
plt.show()
# Coordenadas del camino
# Extrae las trayectorias
x_path = []
y_path = []
paths = stream.lines.get_paths()
for path in paths:
    vertices = path.vertices
    x_path.extend(vertices[:, 0])
    y_path.extend(vertices[:, 1])
#print(x_path)
#print(y_path)
grid = np.zeros((rows, cols), dtype=int)
grid = maps(grid, 2)
#Graficamos sobre el mapa binario
cmap = plt.get_cmap('binary')
plt.imshow(grid, cmap=cmap)
plt.plot(x_path, y_path, '-r', label="APF path")
plt.gca().invert_yaxis()
plt.xlabel('Eje X')
plt.ylabel('Eje Y')
plt.title('Camino generado con APF (inicio = ('+str(start[0][1])+', '+str(start[0][0])+'))'+'(final = ('+str(goal[1])+', '+str(goal[0])+'))')
plt.text(start[0][0], start[0][1], 'Inicio', color='red', fontsize=12)
plt.text(goal[0], goal[1], 'Fin', color='red', fontsize=12)
plt.show()

#-----------------Path tracking-----------------------------------
showAnimation = True
cell_size = 0.05    # [m]
# Escalar las coordenadas a metros
GlobalX_metros = np.array(x_path) * cell_size
GlobalY_metros = np.array(y_path) * cell_size
# Binarizar mapa de obstáculos
cmap = plt.get_cmap('binary')

# Trayectoria a seguir
traj = Trajectory()
traj.generatePath(GlobalX_metros, GlobalY_metros)
# Perfil de velocidad
pathv = 0.25    # [m/s]
T = 100.0       # Tiempo máximo de simulación
# Condiciones iniciales
ack = Ackermann_robot(x0=start[0][1]*cell_size, y0=start[0][0]*cell_size, theta0=np.pi/2, v0=0.0, C=C)
time = 0.0

lastIndex = len(GlobalX_metros)-58
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
plt.title('Seguimiento de trayectoria APF (inicio = ('+str(start[0][1]*cell_size)[:4]+', '+str(start[0][0]*cell_size)[:4]+'))'+'(final = ('+str(goal[1]*cell_size)[:4]+', '+str(goal[0]*cell_size)[:4]+'))')
plt.text(start[0][0], start[0][1], 'Inicio', color='red', fontsize=12)
plt.text(goal[0]*0.05, goal[1]*0.05, 'Fin', color='red', fontsize=12)
plt.show()
#-----------------------------------------------------------------
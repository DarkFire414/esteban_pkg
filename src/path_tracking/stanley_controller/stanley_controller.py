"""
    Stanley: Path tracking controller
"""

"""
TODO
    
"""
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import math
import numpy as np
import matplotlib.pyplot as plt

import anim.draw as draw
from path_tracking.stanley_controller.sim_params import C
from path_tracking.stanley_controller.ackermann_model import Ackermann_robot, States
from path_tracking.stanley_controller.control import Control
from path_tracking.stanley_controller.trajectory import Trajectory    

def main():
    showAnimation = False
    # Se define la trayectoria
    # -------------------------------------------------------------
    # Función sin()
    #pathx = np.arange(0.2, 3, 0.05)                     # [m]
    #pathy = [math.sin(2*ix-0.8) + 1 for ix in pathx]    # [m]
    
    # Trayectoria definida por via points
    ax = [0.0, 1.0, 1.5, 2.0, 3.0]
    ay = [0.0, 0.0, 1.5, 1.5, 0.0]
    traj = Trajectory()
    traj.generatePathViap(ax, ay)
    #traj.load_from_txt(C.path + "/saved_trajectories/traj.txt")
    #pathx, pathy, _cyaw, _ck, _s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.001)

    # Cambio de dirección
    #ax = [0.0, 1.25, 1.75, 3.0]
    #ay = [0.0, 0.0, 0.5, 0.5]
    #pathx, pathy, _cyaw, _ck, _s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
 
    # -------------------------------------------------------------
    # Obtener la orientación de la trayectoria usando la primera derivada
    #dx = np.gradient(pathx)
    #dy = np.gradient(pathy)
    #pathTheta = np.arctan2(dy, dx)

    # -------------------------------------------------------------
    # Perfil de velocidad
    pathv = 0.25     # [m/s]

    T = 100.0   # Tiempo máximo de simulación

    # Condiciones iniciales
    ack = Ackermann_robot(x0=0.0, y0=0.0, theta0=0.0, v0=0.0, C=C)  
    time = 0.0 

    lastIndex = len(traj.px) - 1  # En el último índice se detiene la simulación
    states = States()
    states.addState(time, ack, 0.0)
    control = Control(C)
    target_ind, _ = control.calc_target_index(ack, traj.px, traj.py)

    theta_old = 0.0

    while T >= time and lastIndex > target_ind:
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
            plt.plot(traj.px, traj.py, ".r", label="Trayectoria")
            plt.plot(states.x, states.y, "-b", label="Seguimiento")
            plt.plot(traj.px[target_ind], traj.py[target_ind], "xg", label="target")
            
            draw.draw_car(ack.x, ack.y, ack.theta, steer, C)

            plt.legend()
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[m/s]:" + str(ack.v)[:4])
            plt.pause(0.001)
    
    MSE = np.square(np.subtract(0, states.cross_track_error)).mean() 
    RMSE = math.sqrt(MSE)
    print("Root Mean Square Error:\n")
    print(RMSE)

    print("Saving to txt")
    states.save_to_txt(C.path + "/sim_results/results.txt")

    plt.plot(traj.px, traj.py, ".r", label="Trayectoria")
    plt.plot(states.x, states.y, "-b", label="Seguimiento")
    plt.legend()
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.title("Resultados del seguimiento k = 4.0 [1/s] v = 0.25m/s")
    plt.grid(True)

    plt.subplots(1)
    plt.plot(states.t, states.v, "-r", label="v(t)")
    plt.axhline(y = 0.25, color="b", linestyle="--", label="Velocidad deseada")
    plt.xlabel("tiempo [s]")
    plt.ylabel("v [m/s]")
    plt.title("Velocidad")
    plt.legend()
    plt.grid(True)

    plt.subplots(1)
    plt.plot(states.t, states.cross_track_error, "-r")
    plt.xlabel("Time[s]")
    plt.ylabel("efa")
    plt.grid(True)

    plt.subplots(1)
    plt.plot(states.t, [igamma*180/np.pi for igamma in states.gamma], "-r")
    plt.xlabel("Time[s]")
    plt.ylabel("gamma")
    plt.grid(True)

    plt.subplots(1)
    plt.plot(traj.px, traj.py, "-b", label="bspline")
    plt.plot(ax, ay, ".r", label="Puntos de vía")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.title("Trayectoria")
    plt.legend()
    plt.grid(True)

    plt.show()

if __name__ == '__main__':
    print("Stanley")
    main()
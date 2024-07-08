#!/usr/bin/env python3

import numpy as np

import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
#Funciones para generar los obstaculos o ponerlos en el mapa
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

from obstacles.Obstacles import *
from maps.maps import *
from inflate.inflate import *
import re
#Mostrar el mapa
import matplotlib.pyplot as plt

#Variables de cada obstaculo
obstacleInfo = ""
def obstaclesCallback(obstacle_msg):
    """
    Recibe las coordenadas de los obstaculos detectados por la camara
    Args:
        obstacle_msg (Vector3Stamped): posicion(x,y) y diametro de los obstáculos
    """
    global obstacleInfo
    '''X = obstacle_msg.vector.x
    Y = obstacle_msg.vector.y
    diametro = obstacle_msg.vector.z'''
    obstacleInfo = obstacle_msg.data

def extrae_datos_circulos(obstaculosString):
    patron = r"Circulo (\d+): Distancia \(x=([-+]?\d*\.\d+|\d+), y=([-+]?\d*\.\d+|\d+)\), diametro=([-+]?\d*\.\d+|\d+)"
    datos_circulos = re.findall(patron, obstaculosString)
    datos_circulos = [(int(num_circulo), float(x), float(y), float(diametro)) for num_circulo, x, y, diametro in datos_circulos]
    #print(datos_circulos)
    return datos_circulos


def main():
    global obstacleInfo
    X = 0
    Y = 0
    diametro = 0
    rospy.init_node('Map_Generator', anonymous=True)
    #rospy.Subscriber('/distancias_topic', Vector3Stamped, obstaclesCallback)
    rospy.Subscriber('/distancias_topic', String, obstaclesCallback)
    rate = rospy.Rate(1000) # Hz
    last_time = rospy.Time.now()
    #Mapa por defecto para que la publicacion del mapa comience con él
    rows = 37
    cols = 30
    grid = np.zeros((rows, cols), dtype=int)
    pad_width = ((0,0), (0,7))
    grid = maps(grid, 3)                    #Mapa con solo paredes
    gridINFLATED_MAP = inflate_map(grid, 2) #Mapa inicial inflado
    grid = np.pad(grid, pad_width=pad_width, mode='constant', constant_values=0)
    gridINFLATED_MAP = np.pad(gridINFLATED_MAP, pad_width=pad_width, mode='constant', constant_values=0)
    cmap = plt.get_cmap('binary')
    while True:
        #Extramos la informacion obtenida de los obstaculos
        listaObstaculos = extrae_datos_circulos(obstacleInfo)
        obstaculosOriginales = staticCircularObstacles(listaObstaculos, 0) #Diametro, celdas extra
        obstaculosInflados   = staticCircularObstacles(listaObstaculos, 2) #Diametro, celdas de inflacion
        grid_escenario       = add_obstacles_to_map(np.copy(grid), obstaculosOriginales, listaObstaculos)            #Añade los obstaculos originales al mapa original
        grid_escenarioI      = add_obstacles_to_map(np.copy(gridINFLATED_MAP), obstaculosInflados, listaObstaculos) #Añade los obstaculos inflados al mapa inflado (reduce tiempo de calculo al evitar inflar todo el mapa nuevamente)
        #Mostramos el mapa
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        
        plt.imshow(grid_escenario, cmap=cmap, extent=[0, 1.5, 0, 1.85], origin='lower')
        plt.pause(0.001)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

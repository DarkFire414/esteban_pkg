#!/usr/bin/env python3

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

import numpy as np

#Funciones para generar los obstaculos o ponerlos en el mapa
from obstacles.Obstacles import circularObstacle, staticCircularObstacle, staticCircularObstacles, robotbox
from maps.maps import maps, add_obstacle_to_map, add_obstacles_to_map, add_robotbox_to_map
from inflate.inflate import inflate_map
import re

#Variables de cada obstaculo
obstacleInfo = ""
x0, y0, theta0 = 0, 0, 0
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

def update_initial_pose(msg):
    """
    Actualiza la posicion del robot para
    colocar las celdas a su alrededor

    Args:
        msg: Odometry
    """
    global x0, y0, theta0
    x0 = msg.pose.pose.position.x
    y0 = msg.pose.pose.position.y
    theta0 = msg.pose.pose.orientation.z

def extrae_datos_circulos(obstaculosString):
    """
    Obtiene la información de los obstáculos (ubcación y tamaño)
    a partir de un patrón en una cadena de texto.

    Args:
        obstaculosString (string): Cadena de texto
    Returns:
        datos_circulos
    """
    patron = r"Circulo (\d+): Distancia \(x=([-+]?\d*\.\d+|\d+), y=([-+]?\d*\.\d+|\d+)\), diametro=([-+]?\d*\.\d+|\d+)"
    datos_circulos = re.findall(patron, obstaculosString)
    datos_circulos = [(int(num_circulo), float(x), float(y), float(diametro)) for num_circulo, x, y, diametro in datos_circulos]
    #print(datos_circulos)
    return datos_circulos

def occupancy_grid_publisher(is_simulation, width, height, resolution, rate, wall_inflation, obstacle_inflation):
    """
    Genera y obtiene el costmap a partir de los obstáculos detectados.

    Args:
        is_simulation (boolean): 1-> Modo simulación, 0-> Modo implementación
        width (int): Ancho del mapa [celdas] (columnas)
        height (int): Alto del mapa [celdas] (filas)
        resolution (float): Resolución [m/celda]
        rate (int): Frecuencia de actualización [Hz]
    """
    pub = rospy.Publisher('local_costmap/costmap', OccupancyGrid, queue_size=10)
    pub2 = rospy.Publisher('local_costmap/costmap_inflated', OccupancyGrid, queue_size=10) 
    rospy.Subscriber('/distancias_topic', String, obstaclesCallback)
    rospy.Subscriber('/initial_pose', Odometry, update_initial_pose)

    rate = rospy.Rate(rate)

    """
    La forma del mapa debe ajustarse a un cuadrado, debido a que cualquier 
    otra forma genera problemas al aplicar el np.reshape()
    El valor más grande entre ancho y alto será el que defina el tamaño 
    final del mapa (por lado)
    """
    side_map = np.max([width, height])

    # Crear un mensaje de tipo OccupancyGrid
    occupancy_grid_msg = OccupancyGrid()
    occupancy_grid_msg.header.frame_id = "odom"
    occupancy_grid_msg.info.width = side_map
    occupancy_grid_msg.info.height = side_map
    occupancy_grid_msg.info.resolution = resolution
    # Localización en el plano xy de la esquina superior izquierda del mapa
    occupancy_grid_msg.info.origin.position.x = 0.0
    occupancy_grid_msg.info.origin.position.y = 0.0
    occupancy_grid_msg.info.origin.position.z = 0.0
    
    # Rotación del mapa
    roll  = 0 # X
    pitch = 0 # Y
    yaw   = 0 # Z
    quaternion = quaternion_from_euler(roll, pitch, yaw)

    occupancy_grid_msg.info.origin.orientation.x = quaternion[0]
    occupancy_grid_msg.info.origin.orientation.y = quaternion[1]
    occupancy_grid_msg.info.origin.orientation.z = quaternion[2]
    occupancy_grid_msg.info.origin.orientation.w = quaternion[3]

    # Crear un mensaje de tipo OccupancyGrid
    occupancy_grid_msg_inflated = OccupancyGrid()
    occupancy_grid_msg_inflated.header.frame_id = "odom"
    occupancy_grid_msg_inflated.info.width = side_map
    occupancy_grid_msg_inflated.info.height = side_map
    occupancy_grid_msg_inflated.info.resolution = resolution
    # Localización en el plano xy de la esquina superior izquierda del mapa
    occupancy_grid_msg_inflated.info.origin.position.x = 0.0
    occupancy_grid_msg_inflated.info.origin.position.y = 0.0
    occupancy_grid_msg_inflated.info.origin.position.z = 0.0

    occupancy_grid_msg_inflated.info.origin.orientation.x = quaternion[0]
    occupancy_grid_msg_inflated.info.origin.orientation.y = quaternion[1]
    occupancy_grid_msg_inflated.info.origin.orientation.z = quaternion[2]
    occupancy_grid_msg_inflated.info.origin.orientation.w = quaternion[3]

    # Generar mapa
    global obstacleInfo
    global x0, y0, theta0

    # Crear una matriz de datos de ocupación (dummy data)
                    # rows, cols
    grid = np.zeros((height, width), dtype=int)

    # Se compensa el tamaño del grid para que sea cuadrado
    arriba = 0      # Filas a agregar a arriba
    derecha = 0     # Columnas a agregar a la derecha
    if width > height: 
        arriba = width-height
    elif height > width:
        derecha = height-width

    #   ((arriba, abajo), (izquierda, derecha))
    pad_width = ((arriba,0), (0,derecha))

    if is_simulation:
        grid = maps(grid, 2)                        # Mapa con obstáculos predefinidos
    else:
        grid = maps(grid, 3)                        # Mapa con solo paredes
        
    grid = np.pad(grid, pad_width=pad_width, mode='constant', constant_values=0)

    # Ajusta el valor de 0-1 a 0-100 para rviz (100 -> Celda ocupada)
    occupancy_data = grid*100                       
    occupancy_grid_msg.data = occupancy_data.flatten('C').tolist()

    inflated_grid = inflate_map(grid, wall_inflation)
    
    occupancy_data_inflated = inflated_grid*100
    occupancy_grid_msg_inflated.data = occupancy_data_inflated.flatten('C').tolist()

    grid_escenario           = np.copy(grid)
    grid_escenarioI          = np.copy(inflated_grid)
    
    if is_simulation:
        print("COSTMAP: Running... (Simulation)")
    else:
        print("COSTMAP: Running...")

    # Bucle de publicación
    while not rospy.is_shutdown():
        # Actualizar la marca de tiempo del mensaje
        occupancy_grid_msg.header.stamp = rospy.Time.now()
        occupancy_grid_msg_inflated.header.stamp = rospy.Time.now()
        #Caja del robot
        cajaRobot            = robotbox(theta0)
        if not is_simulation:
            #Extramos la informacion obtenida de los obstaculos
            listaObstaculos      = extrae_datos_circulos(obstacleInfo)
            obstaculosOriginales = staticCircularObstacles(listaObstaculos, 0) #Diametro, celdas extra
            obstaculosInflados   = staticCircularObstacles(listaObstaculos, obstacle_inflation) #Diametro, celdas de inflacion

            #Añade los obstaculos junto con la caja del robot, la cual permanece estatica durante la navegacion
            grid_escenario       = add_obstacles_to_map(np.copy(grid), obstaculosOriginales, listaObstaculos)        #Añade los obstaculos originales al mapa original
            grid_escenarioI      = add_obstacles_to_map(np.copy(inflated_grid), obstaculosInflados, listaObstaculos) #Añade los obstaculos inflados al mapa inflado (reduce tiempo de calculo al evitar inflar todo el mapa nuevamente)
            if cajaRobot is not None:
                grid_escenario       = add_robotbox_to_map(np.copy(grid_escenario), cajaRobot, x0, y0)        #Añade los obstaculos originales al mapa original
                grid_escenarioI      = add_robotbox_to_map(np.copy(grid_escenarioI), cajaRobot, x0, y0)        #Añade los obstaculos originales al mapa original
        else:
            # Añadimos unicamente la caja generada del robot
            if cajaRobot is not None:
                grid_escenario       = add_robotbox_to_map(np.copy(grid), cajaRobot, x0, y0)        #Añade los obstaculos originales al mapa original
                grid_escenarioI      = add_robotbox_to_map(np.copy(inflated_grid), cajaRobot, x0, y0)        #Añade los obstaculos originales al mapa original

        # Actualizar datos del mensaje
        occupancy_data = grid_escenario*100
        occupancy_grid_msg.data = occupancy_data.flatten('C').tolist()

        occupancy_data_inflated = grid_escenarioI*100
        occupancy_grid_msg_inflated.data = occupancy_data_inflated.flatten('C').tolist()

        # Publicar el mensaje
        pub.publish(occupancy_grid_msg)
        pub2.publish(occupancy_grid_msg_inflated)

        # Esperar según la frecuencia especificada
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('occupancy_grid_node', anonymous=True)

    is_simulation = rospy.get_param('/is_simulation')
    cols = rospy.get_param('/costmap/cols')
    rows = rospy.get_param('/costmap/rows')
    resolution = rospy.get_param('/costmap/resolution')
    rate = rospy.get_param('/costmap/rate')
    wall_inflation = rospy.get_param('/costmap/wall_inflation')
    obstacle_inflation = rospy.get_param('/costmap/obstacle_inflation')

    try:
        occupancy_grid_publisher(is_simulation, cols, rows, resolution, rate, wall_inflation, obstacle_inflation)
    except rospy.ROSInterruptException:
        pass

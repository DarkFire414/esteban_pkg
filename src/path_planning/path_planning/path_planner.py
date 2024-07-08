#!/usr/bin/env python3

import pathlib
import sys
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import numpy as np
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from HybridAstar import HybridAstar
from HybridDijkstra import Dijkstra
from APF import APF3

class Path_Planner:
    def __init__(self, algortimo, is_simulation):
        """
        Planeador de ruta.

        Args:
            algoritmo (string): A->A*, D->Dijkstra, C->APF
            is_simulation (boolean): 1-> Modo simulación, 0-> Modo implementación
        """
        # Pose de inicio (celda)
        self.x0 = 0
        self.y0 = 0
        self.theta0 = 0

        # Pose deseada (celda)
        self.xd = 0
        self.yd = 0
        self.thetad = 0.0

        # Mapa
        self.grid = np.array([], dtype=int)
        self.rows = 0
        self.cols = 0
        self.resolution = 0.0

        # Camino suavizado
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Camino original
        self.raw_path = Path()
        self.raw_path.header.frame_id = 'odom'

        #Algoritmo que va a usarse para planear la ruta
        self.algoritmo = algortimo

        #Actualiza los obstaculos para el algoritmo APF de ser necesario
        self.is_simulation = is_simulation
        self.obstacleInfo = ""

        #Indica si se actualiza o no la caja del robot
        self.actualiza = True

        #Publica la posicion del carro necesaria para orientar la ruta generada
        self.odom_msg = Odometry()

        #Almacena las coordenadas devueltas por los algoritmos de generacion de camino
        self.Glob_x = []
        self.Glob_y = []
        self.Glob_t = []

        #listas para almacenar en un solo archivo las metricas
        self.longitud_del_camino = []
        self.tiempo_calculo = []

        #Iteraciones para las pruebas automatizadas
        self.iteracion = 0

        """
        Es necesario subscribirse a los tópicos después de definir los atributos de la clase 
        debido a que los callbacks pueden ejecutarse antes de que se terminen de definir.
        """
        self.pub   = rospy.Publisher('/path/smooth', Path, queue_size=10)
        self.pub2  = rospy.Publisher('/path/raw', Path, queue_size=10)
        self.pub3  = rospy.Publisher('/initial_pose', Odometry, queue_size=10)

        self.subs  = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.update_goal)
        self.subs2 = rospy.Subscriber('/local_costmap/costmap_inflated', OccupancyGrid, self.update_map)
        self.subs3 = rospy.Subscriber('/odom', Odometry, self.update_initial_pose)
        self.subs4 = rospy.Subscriber('/cmd_vel', Twist, self.robotReachedGoal)
        #self.subs4 = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
    
    def update_goal(self, msg):
        """
        Recibe el punto objetivo (posición y orientación)
        La posición es recibida en coordenadas (x, y, z) [m]
        La orientación es un cuaternión (x, y, z, w)

        Arg:
            msg (PoseStamped)
        """
        #Extrae las coordenadas del destino deseadas
        self.actualiza = False
        self.xd, self.yd = self.meters_to_cells(msg.pose.position.x, msg.pose.position.y)
        self.iteracion = self.iteracion + 1
        #orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w] 

        """
        Comprueba si el punto inicial está fuera del mapa
        """
        if (self.y0 > self.rows) or (self.x0 > self.cols) or (self.y0 < 0) or (self.x0 < 0):
            print('PATH_PLANNER: The robot is outside the map')
            return
        
        """
        Comprueba si el objetivo está fuera del mapa
        """
        if (self.yd > self.rows) or (self.xd > self.cols) or (self.yd < 0) or (self.xd < 0):
            print('PATH_PLANNER: The goal is outside the map')
            return
        
        """
        Comprueba si el punto inicial está sobre alguna celda ocupada
        """
        cell_status = self.grid[self.y0][self.x0]
        if cell_status > 50.0:
            # La celda está ocupada
            print(f'PATH_PLANNER: Cell ocupied, please modify the initial pose')
            return
        
        """
        Comprueba si el objetivo está en alguna celda ocupada
        """
        cell_status = self.grid[self.yd][self.xd]
        if cell_status > 50.0:
        # La celda está ocupada
            print(f'PATH_PLANNER: Goal can not be an ocupied cell')
            return

        #(roll, pitch, yaw) = euler_from_quaternion(orientation)
        #self.thetad = yaw

        print(f'PATH_PLANNER: Setting goal to ({self.xd}, {self.yd}) [x,y][cells]')
        print(f'PATH_PLANNER: Generating path')
        start = rospy.Time.now().to_sec()
        self.generate_path()
        end = rospy.Time.now().to_sec()
        #Calculamos el angulo de giro en cada momento de la navegacion para determinar el riesgo de la misma
        self.riskNavigation(self.Glob_t)
        #Obtenemos la metrica de la longitud del camino generado
        self.longitud_del_camino.append(self.pathLength(self.Glob_x, self.Glob_y))
        self.tiempo_calculo.append(end-start)
        print("La longitud del camino es: ", self.pathLength(self.Glob_x, self.Glob_y))
        print(f'PATH_PLANNER: Total path planning time: {end-start}')
        if self.is_simulation and self.iteracion >= 5:
            file_path = str(pathlib.Path(__file__).parent) + "/saved_data/DijkstraMAPA3"
            self.save_metrics_to_txt(file_path)


    def update_map(self, msg):
        """
        Recibe el mapa y lo transforma a un grid 2D

        Args:
            msg (OccupancyGrid)
        """
        self.cols = msg.info.width
        self.rows = msg.info.height
        self.resolution = msg.info.resolution
        data_flatten = np.array(msg.data)
        self.grid = data_flatten.reshape(self.rows, self.cols)
    
    def update_initial_pose(self, msg):
        x0_meters = msg.pose.pose.position.x
        y0_meters = msg.pose.pose.position.y
        #orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w] 
        #(roll, pitch, yaw) = euler_from_quaternion(orientation)
        self.theta0 = msg.pose.pose.orientation.z
        if self.theta0 < 0:
            self.theta0 += np.pi*2
        self.x0, self.y0 = self.meters_to_cells(x0_meters, y0_meters)
        
        if self.actualiza == True and self.algoritmo != "C":
            #Publicamos la pose inicial del carrito para colocar los obstaculos a su al rededor
            self.odom_msg.pose.pose.position.x = self.x0
            self.odom_msg.pose.pose.position.y = self.y0
            self.odom_msg.pose.pose.orientation.z = self.theta0
            self.pub3.publish(self.odom_msg)

    def robotReachedGoal(self, msg):
        if msg.linear.x <= 0.1: #El robot esta detenido entonces debe actualizar la posicion
            self.actualiza = True

    def generate_path(self):
        """
        Publica el camino generado por el planeador de ruta
        en forma de puntos bajo el tópico /path
        """
        # Origen y destino [cells]
        origin = (self.y0, self.x0)
        goal = (self.yd, self.xd)

        if self.algoritmo == "A":   #Generamos la ruta con A*
            Global_Astar_Planner = HybridAstar.AstarAckermann(origin, goal, self.grid)
            GlobalRawPath, GlobalPath = Global_Astar_Planner.astar()
            #GlobalRawY, GlobalRawX = zip(*GlobalRawPath)
            GlobalY, GlobalX, GlobalTheta = zip(*GlobalPath)
            self.Glob_x = GlobalX
            self.Glob_y = GlobalY
            self.Glob_t = GlobalTheta

        elif self.algoritmo == "D": #Generamos la ruta con Dijkstra
            Global_Dijkstra_Planner = Dijkstra.GlobalDijkstraAckermann(origin, goal, self.grid)
            GlobalRawPath, GlobalPath = Global_Dijkstra_Planner.dijkstra()
            #GlobalRawY, GlobalRawX = zip(*GlobalRawPath)
            GlobalY, GlobalX, GlobalTheta = zip(*GlobalPath)
            self.Glob_x = GlobalX
            self.Glob_y = GlobalY
            self.Glob_t = GlobalTheta

        elif self.algoritmo == "C": #Generamos la ruta con campos potenciales
            grid_size = 1.0         # potential grid size [m]
            robot_radius = 1        # robot radius [m]        
            Global_Local_APF_Planner = APF3.APF_Ackermann(origin, goal, grid_size, robot_radius, self.grid, self.theta0)
            GlobalY, GlobalX, GlobalTheta = zip(*(Global_Local_APF_Planner.potential_field_planning()))
            self.Glob_x = GlobalX
            self.Glob_y = GlobalY
            self.Glob_t = GlobalTheta

        self.path.poses = self.coords_to_poses_array(GlobalX, GlobalY, GlobalTheta)
        #self.raw_path.poses = self.coords_to_poses_array(GlobalRawX, GlobalRawY, None)

        self.pub.publish(self.path)
        #self.pub2.publish(self.raw_path)
    
    def coords_to_poses_array(self, xarr, yarr, thetaarr):
        """
        Convierte arreglos x, y, theta de poses de ruta a un arreglo de 
        PosesStamped de ROS necesario para un mensaje tipo Path

        Args:
            xarr (float array): Arreglo de coordenadas x
            yarr (float array): Arreglo de coordenadas y
            thetaarr (float array): Arreglo de ángulos theta
        
        Returns
            poses (array): 
                poses = [ [ x0, y0, theta0 ], [ x1, y1, theta1, ]...]
        """

        theta_is_given = thetaarr
        poses = []
        for i in range(0, len(xarr), 1):
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.z = 0.0
            pose.pose.position.x = xarr[i]*self.resolution
            pose.pose.position.y = yarr[i]*self.resolution
            if theta_is_given is not None:
                # x y z w                          Roll Pitch Yaw
                quaternion = quaternion_from_euler(0, 0, thetaarr[i])
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
            poses.append(pose)
        return poses

    def meters_to_cells(self, xm, ym):
        """
        Convierte una coordenada en metros a su equivalente en celdas

        Args:
            xm (float): Coordenada x en metros
            ym (float): Coordenada y en metros
        
        Returns:
            [xc, yc] (float, float): Coordenadas en celdas 
        """
        if self.resolution > 0.0:
            xc = int(round(xm / self.resolution))
            yc = int(round(ym / self.resolution))
            return xc, yc
        else: 
            return 0, 0
    #Funciones para obtener los parametros de comparacion
    def riskNavigation(self, globalT):
        #Evaluacion de la seguridad en la ruta
        differences = [y - x for x, y in zip(globalT[:-1], globalT[1:])] #Cambios en radianes/punto en la ruta generadax
        maximo_cambio = max(differences)
        minimo_cambio = min(differences)
        if (maximo_cambio >= (30*np.pi)/180 or abs(minimo_cambio) >= (30*np.pi)/180): #Ruta con altas posibilidades de colision dado que la ruta supera las capacidades fisicas del robot
            print("Ruta generada con altas probabilidades de colisión")
    #funcion para calcular la longitud del camino generado
    def pathLength(self, globalX, globalY):
        pathlength = 0
        for i in range(len(globalX)-1):
            pathlength += math.sqrt((globalX[i+1] - globalX[i])**2 + (globalY[i+1] - globalY[i])**2)
        return pathlength*5 #conversion de celdas a cm

    def save_metrics_to_txt(self, file_name):
        """
        Guarda los estados en un archivo txt con el formato:
            longitud camino   tiempo de calculo ...
            ...
            ...
        Args:
            file_name (string): Ruta del archivo a guardar <sin extensión> ex: /home/src/control/resultados
        """
        with open(file_name + "path_planner_data.txt", 'w') as file:
            # Escribir encabezado
            file.write("iteracion\tlongitud\ttiempo calculo\n")
            for i in range(len(self.longitud_del_camino)):
                # Escribir una línea de datos en el archivo
                longitud = self.longitud_del_camino[i]
                tiempo_calculo = self.tiempo_calculo[i]
                file.write("{:.4f}\t{:.4f}\t{:.4f}\n".format(i, longitud, tiempo_calculo))

          

def main(args):
    rospy.init_node('path_planner_node')
    algoritmo = rospy.get_param('/algoritmo')
    is_simulation = rospy.get_param('/is_simulation')
    path_planner = Path_Planner(algoritmo, is_simulation)
    print('PATH_PLANNER: Waiting for goal point...')
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
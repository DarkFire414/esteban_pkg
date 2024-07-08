#!/usr/bin/env python3

import numpy as np
from control.stanley_controller.ackermann_states import Ackermann_States 

import rospy
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def normalize_angle(angle):
    """
    Normaliza un ańgulo entre los valores [-pi, pi].

    Args:
        angle (float):
    
    Returns:
        (float): Ángulo en radianes entre [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

class Stanley_Controller:
    def __init__(self, G, WB, rate):
        """
        Controlador de seguimiento de trayectoria Stanley

        Args:
            G (Dictionary): Diccionario con las ganancias del controlador ex. G = { 'k':10.0, 'kv':1.0 }
            WB (float): Distancia medida desde el eje trasero al eje delantero del vehículo
            rate (int): Frecuancia de actualización [Hz]
        """
        self.sub = rospy.Subscriber('/path/smooth', Path, self.update_path)
        self.subs = rospy.Subscriber('/odom', Odometry, self.update_position)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.velocity_message = Twist()

        self.G = G
        self.WB = WB
        self.rate = rospy.Rate(rate) # Hz

        self.pathx = []
        self.pathy = []
        self.pathyaw = []

        # Perfil de velocidad
        self.pathv = 0.25   # [m/s]

        # Condiciones iniciales
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.stop()
        
        # Detiene al robot, 
        #   0 -> el robot continua siguiento la trayectoria actual
        #   1 -> el robot se detiene
        self.stop_robot = 0

        self.Acks = Ackermann_States()

    def stop(self):
        """
        Envía una velocidad y ángulo gamma 0
        """
        self.velocity_message.linear.x = 0
        self.velocity_message.linear.z = 0
        self.pub.publish(self.velocity_message)
    
    def clearPath(self):
        self.pathx.clear()
        self.pathy.clear()
        self.pathyaw.clear()
    
    def update_position(self, odom_mesaage):
        """
        Función callback que se ejecuta cada que llega un nuevo mensaje
        en el tópico de odometría, recibe la posición del robot y su 
        orientación y actualiza los atributos del objeto.

        Args:
            odom_message (nav_msgs/msg/Odometry)
        """
        self.x = odom_mesaage.pose.pose.position.x
        self.y = odom_mesaage.pose.pose.position.y
        self.theta = odom_mesaage.pose.pose.orientation.z

    def update_path(self, msg):
        """
        Actualiza la ruta que está utilizando el controlador

        Args:
            msg (nav_msgs/msg/Path)
        """
        self.pathx, self.pathy, self.pathyaw = self.poses_array_to_coords(msg.poses)
        
    def poses_array_to_coords(self, poses):
        """
        Extrae las coordenadas x, y, theta de la ruta desde un mensaje tipo 
        Path de ROS

        Args:
            msg (nav_msgs/msg/Path)
        """
        xc = []
        yc = []
        yaw = []
        for pose_stamped in poses:
            xc.append(pose_stamped.pose.position.x)
            yc.append(pose_stamped.pose.position.y)
            # La orientación se recibe en quateriniones
            orientation = [
                pose_stamped.pose.orientation.x,
                pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z,
                pose_stamped.pose.orientation.w
            ] 
            # Se convierte a águlos de euler
            (roll, pitch, yw) = euler_from_quaternion(orientation)
            yaw.append(yw)
        return xc, yc, yaw

    def stanley_control(self, cx, cy, cyaw, last_target_idx):
        """
        Control de seguimiento de trayectoria Stanley.

        Args:
            cx (float array): Puntos en x de la trayectoria [m]
            cy (float array): Puntos en y de la trayectoria [m]
            cyaw (float array): Orientación para cada punto en la trayectoria [rad]
            last_target_idx (int): último índice al que el vehículo se dirigió
        
        Returns:
            delta (float): Steering angle
            current_target_idx (int): índice del punto de la trajectoria al que debe dirigirse
            error_fron_axle (float): error lateral
        """
        # Obtiene el índice el punto al cuál dirigirse y el error lateral
        current_target_idx, error_front_axle = self.calc_target_index(cx, cy)
        
        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = normalize_angle(cyaw[current_target_idx] - self.theta)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(self.G['k'] * error_front_axle, self.G['kv'] + self.pathv)

        # Steering control
        delta = theta_e + theta_d

        return delta, current_target_idx, error_front_axle
    
    def calc_target_index(self, cx, cy):
        """
        Obtiene el índice sobre la trayectoria al cuál el 
        vehículo debe dirigirse

        Args:
            cx (float array): Puntos en x de la trayectoria [m]
            cy (float array): Puntos en y de la trayectoria [m]
        
        Return:
            target_idx (int): ínice objetivo
            error_front_axl (float): Error lateral
        """
        # Calc front axle position
        fx = self.x + self.WB * np.cos(self.theta)
        fy = self.y + self.WB * np.sin(self.theta)

        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)                               

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(self.theta + np.pi / 2),
                          -np.sin(self.theta + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle
    
    def run(self, file_name=None):
        """
        Ejecuta el controlador se seguimiento de trayectoria hasta que ROS sea 
        detenido o el robot sea detenido con la variable self.stop_robot

        Args:
            file_name (string): Ruta del archivo 
        """
        self.stop_robot = 0
        self.clearPath()

        while not rospy.is_shutdown() and not self.stop_robot:
            if len(self.pathx) > 1:              # Comprueba que se haya recibido una ruta
                lastIndex = len(self.pathx) - 1  # En el último índice se detiene el algoritmo

                # Obtiene el indice inicial
                target_ind, _ = self.calc_target_index(self.pathx, self.pathy)

                start = rospy.Time.now().to_sec()
                t0 = rospy.Time.now().to_sec()
                t = 0.0         # [s]                
                while not self.stop_robot and lastIndex > target_ind:
                    gamma_i, target_ind, efa = self.stanley_control(self.pathx, self.pathy, self.pathyaw, target_ind)
                    
                    self.velocity_message.linear.x = self.pathv
                    self.velocity_message.linear.z = gamma_i

                    self.pub.publish(self.velocity_message)

                    t = rospy.Time.now().to_sec() - t0
                    self.Acks.addState(t, self.x, self.y, self.theta, self.pathv, gamma_i, efa)
                
                    self.rate.sleep()
                
                self.stop()
                end = rospy.Time.now().to_sec()
                print(f'\n\t PATH_TRACKING: Total travel time (s): {round(end-start, 4)}')

                if file_name is not None:
                    self.Acks.set_path(self.pathx, self.pathy, self.pathyaw)
                    self.Acks.save_data_to_txt(file_name)
                    self.Acks.save_path_to_txt(file_name)
                    print("\n\t PATH_TRACKING: File saved")

                self.clearPath()
                break
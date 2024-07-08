#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion

class Ackermann_Model:
    def __init__(self, gamma_saturation, x0, y0, theta0, gamma0, v0, L, frecuency_update):
        """
        Modelo cinemático de bicicleta para un robot móvil tipo Ackermann

        Parameters
        ----------
        gamma_saturation : float
            Valor máximo que puede tomar la dirección del vehículo. [rad]
        x0 : float
            Posición inicial en x [m]
        y0 : float
            Posición inicial en y [m]
        theta0 : float
            Orientación incial en z (yaw) [rad]
        gamma0 : float
            Ángulo inicial de la dirección [rad]
        v0 : float
            Velocidad inicial [m/s]
        L : float
            Longitud medida desde el eje trasero al eje delantero del vehículo [m]
        frecuency_update : int
            Frecuencia de actualización [Hz]
        """

        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.subs = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.subs = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.position_callback)
        
        self.updateFecuency = frecuency_update #Hz
        self.rate = rospy.Rate(self.updateFecuency) # Hz
        self.last_time = rospy.Time.now()

        # Características físicas del robot
        #   Valores de saturación
        self.gammaSaturationValue = gamma_saturation
        self.linear_velocitySaturationValueMin = 0.14
        self.linear_velocitySaturationValueMax = 0.25

        #   Lingitud del vehículo de centro a centro de cada rueda
        self.L = L

        self.x = x0
        self.y = y0
        self.theta = theta0
        self.gamma = gamma0

        self.linear_velocity = v0
    
    def position_callback(self, msg):
        """
        Recibe una nueva posición y orientación para el robot
        puede utilizarse para establecer condiciones iniciales
        distintas a cero antes de ejecutar el controlador.

        Args:
            msg (Odometry): Mensaje con la positición y orientación
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w] 
        (roll, pitch, yaw) = euler_from_quaternion(orientation)
        self.theta = yaw

    def cmd_vel_callback(self, msg):
        """
        Recibe la velocidad y ángulo gamma que debe seguir el robot.
        Ambos valores son saturados respecto a los parámetros establecidos
        en el constructor de la clase.
        """
        self.linear_velocity = msg.linear.x
        self.gamma = msg.linear.z
        
        
        if self.linear_velocity < 0.03:
            self.linear_velocity = 0
        else:
            self.linear_velocity = np.clip(self.linear_velocity, self.linear_velocitySaturationValueMin, self.linear_velocitySaturationValueMax)
       
        self.gamma = np.clip(self.gamma, -self.gammaSaturationValue, self.gammaSaturationValue)

    def run(self):
        """
        Ejecuta el modelo de bicicleta de forma indefinida.
        Publica la odometría del robot (x, y, theta) de acuerdo
        a las entradas de velocidad y gamma.
        """
        dt = 1/self.updateFecuency
        odom_msg = Odometry()

        while not rospy.is_shutdown():
            # Ecuaciones diferenciales que definen al modelo de bicicleta
            dx = self.linear_velocity*np.cos(self.theta)
            dy = self.linear_velocity*np.sin(self.theta)
            dtheta = (self.linear_velocity*np.tan(self.gamma))/self.L

            # Integración numérica
            self.x = self.x + dx*dt
            self.y = self.y + dy*dt
            self.theta = self.theta + dtheta*dt

            # Publicar odometría
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.orientation.z = self.theta
            self.pub.publish(odom_msg)

            #print("x = {:.2f}, y = {:.2f}, v = {:.2f}".format(self.x, self.y, self.linear_velocity))

            self.rate.sleep()

def main(args):
    rospy.init_node('ackermann_model_node')

    gamma_saturation = rospy.get_param('/robot/gamma_sat')
    x0 = rospy.get_param('/robot/model/x0')
    y0 = rospy.get_param('/robot/model/y0')
    theta0 = rospy.get_param('/robot/model/theta0')
    gamma0 = rospy.get_param('/robot/model/gamma0')
    v0 = rospy.get_param('/robot/model/v0')
    L = rospy.get_param('/robot/WB')
    frecuency_update = rospy.get_param('/robot/model/rate')

    try:
        ack_model = Ackermann_Model(
            gamma_saturation, 
            x0, y0, theta0, gamma0, v0, 
            L, frecuency_update)
        print("ACK_MODEL: Running...")
        ack_model.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")

if __name__ == '__main__':
    main(sys.argv)
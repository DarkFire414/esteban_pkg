#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class Ackermann_Model:
    """
    Modelo de bicicleta para un robot móvil tipo Ackermann
    """
    def __init__(self):
        rospy.init_node('ackermann_model')
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.subs = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.subs = rospy.Subscriber('/set_position', Odometry, self.position_callback)
        
        self.updateFecuency = 10000 #Hz
        self.rate = rospy.Rate(self.updateFecuency) # Hz
        self.last_time = rospy.Time.now()

        # Características físicas del robot
        #   Valores de saturación
        self.gammaSaturationValue = np.pi/6
        self.linear_velocitySaturationValueMin = 0.14
        self.linear_velocitySaturationValueMax = 0.25

        #   Lingitud del vehículo de centro a centro de cada rueda
        self.L = 0.145

        self.x = 0
        self.y = 0
        self.theta = 0
        self.gamma = 0

        self.linear_velocity = 0
    
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
        self.theta = msg.pose.pose.orientation.z

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
        elif self.linear_velocity > self.linear_velocitySaturationValueMax:
            self.linear_velocitySaturationValueMax
        elif self.linear_velocity < self.linear_velocitySaturationValueMin:
            self.linear_velocity = self.linear_velocitySaturationValueMin
       
        if self.gamma > self.gammaSaturationValue:
            self.gamma = self.gammaSaturationValue
        elif self.gamma < -self.gammaSaturationValue:
            self.gamma = -self.gammaSaturationValue

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

if __name__ == '__main__':
    try:
        ack_model = Ackermann_Model()
        print("ACK_MODEL: Running...")
        ack_model.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
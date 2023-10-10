#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np

class Ackermann_Control:
    """
    Algoritmos de control para un robot móvil tipo Ackermann.

    Args:
        x0 (float): Posición inicial en x del robot.
        y0 (float): Posición inicial en y del robot.
    """
    def __init__(self, x0, y0, theta0):
        rospy.init_node('ackermann_control_node')
        self.subs = rospy.Subscriber('/odom', Odometry, self.update_position)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocity_message = Twist()
        
        # Condiciones iniciales
        self.x = x0
        self.y = y0
        self.theta = theta0

        #   Establece una velocidad 0 inicial
        self.stop()

        # Detener controlador
        self.stopSim = 0

        # Longitud del robot
        self.L = 0.145  # m

        self.xData = []
        self.yData = []
        self.thetaData = []
        self.errorData = []
        self.velocityData = []
        self.timeData = []
    
    def stop(self):
        """
        Envía una velocidad y ángulo gamma 0
        """
        self.velocity_message.linear.x = 0
        self.velocity_message.linear.z = 0
        self.pub.publish(self.velocity_message)
    
    def update_position(self, odom_mesaage):
        """
        Función callback que se ejecuta cada que llega un nuevo mensaje
        en el tópico de odometría, recibe la posición del robot y su 
        orientación y actualiza los atributos del objeto.
        """
        self.x = odom_mesaage.pose.pose.position.x
        self.y = odom_mesaage.pose.pose.position.y
        self.theta = odom_mesaage.pose.pose.orientation.z

    def go_to_goal(self, x_goal, y_goal):
        """
        Mover al robot a un punto en el espacio

        Args:
            x_goal (float): Coordenada x objetivo
            y_goal (float): Coordenada y objetivo
        """
        self.cleanData()

        t0 = rospy.Time.now().to_sec()

        self.stopSim = 0
        rate = rospy.Rate(100) # Hz

        while not self.stopSim and not rospy.is_shutdown():
            self.xData.append(self.x)
            self.yData.append(self.y)
            self.timeData.append(rospy.Time.now().to_sec()-t0)
            self.thetaData.append(self.theta)

            # Constantes de control
            k_lnear = 0.3
            k_angular = 6

            distance = abs(math.sqrt(((x_goal-self.x) ** 2) + ((y_goal-self.y) ** 2)))           
            self.errorData.append(distance)

            linear_speed = distance * k_lnear
            self.velocityData.append(linear_speed)
            
            desired_angle_goal = math.atan2(y_goal-self.y, x_goal-self.x)
            gamma = self.angdiff(self.theta, desired_angle_goal)*k_angular

            self.velocity_message.linear.x = linear_speed
            self.velocity_message.linear.z = gamma

            self.pub.publish(self.velocity_message)

            if (distance<0.1):
                self.velocity_message.linear.x = 0       
                self.pub.publish(self.velocity_message)
                break

            rate.sleep()
            
        self.velocity_message.linear.x = 0       
        self.pub.publish(self.velocity_message)
        self.saveToFile('go_to_goal')
    
    def cleanData(self):
        self.xData.clear()
        self.yData.clear()
        self.timeData.clear()
        self.thetaData.clear()
        self.errorData.clear()
        self.velocityData.clear()

    def saveToFile(self, type):
        """
        Guarda los resultados generados durante el controlador en un archivo de texto.

        Args:
            type (string): tipo de controlador ejecutado, se agrega al nombre del archivo.
        """
        data = list(zip(self.timeData, self.xData, self.yData, self.thetaData, self.errorData, self.velocityData))

        #np.savetxt('/home/joahan/catkin_ws/src/esteban_pkg/data.txt', data, fmt='%f\t%f\t%f')
        
        with open('/home/joahan/catkin_ws/src/esteban_pkg/{}.txt'.format(type), 'wb') as f:
            f.write(b'Time (s)\t x(m)\t y(m)\t theta(rad)\t error\t velocidad(m/s)\n\n')

            np.savetxt(f, data, fmt='%f\t %f\t %f\t %f\t %f\t %f')
        print("ACK_CONTROL: Archivo guardado")
        
    def angdiff(self, ref_angle, comp_angle):
        """
        Obtiene la diferencia entre dos ángulos y ajusta el resultado
        en el intervalo [-pi, pi]

        Args:
            ref_angle (float): Ángulo de referencia en radianes
            comp_angle (float): Ángulo a comparar en radianes
        
        Returns:
            diff (float): Diferencia entre ángulos
        """

        diff = math.atan2(math.sin(comp_angle - ref_angle), math.cos(comp_angle - ref_angle))
        return diff
     
    def go_to_pose(self, x_goal, y_goal, theta_goal):
        """
        Mueve el robot a una pose específica

        Args:
            x_goal (float): Coordenada x objetivo
            y_goal (float): Coordenada y objetivo
            theta_gloat (float): Ángulo en radianes con el que el robot debe llegar al objetivo
        """
        self.cleanData()
        t0 = rospy.Time.now().to_sec()

        # Constantes de control
        k_rho= 0.5
        k_alpha = 2.5
        k_beta = -2

        self.stopSim = 0
        rate = rospy.Rate(100) # Hz

        while not self.stopSim and not rospy.is_shutdown():
            self.xData.append(self.x)
            self.yData.append(self.y)
            self.timeData.append(rospy.Time.now().to_sec()-t0)
            self.thetaData.append(self.theta)

            x_diff = x_goal-self.x
            y_diff = y_goal-self.y

            rho = np.hypot(x_diff, y_diff) 
            alpha = (np.arctan2(y_diff, x_diff) - self.theta + np.pi) % (2 * np.pi) - np.pi
            beta =  (theta_goal - self.theta - alpha + np.pi) % (2 * np.pi) - np.pi
           
            linear_speed = rho * k_rho
            angular_speed = k_alpha*alpha + k_beta*beta

            self.errorData.append(rho)
            self.velocityData.append(linear_speed)

            gamma = np.arctan2((angular_speed*self.L), linear_speed)

            self.velocity_message.linear.x = linear_speed
            self.velocity_message.linear.z = gamma

            self.pub.publish(self.velocity_message)

            if (rho<0.1):
                self.velocity_message.linear.x = 0
                self.pub.publish(self.velocity_message)
                break

            rate.sleep()

        self.velocity_message.linear.x = 0       
        self.pub.publish(self.velocity_message)
        self.saveToFile('go_to_pose')
    
    def follow_path(self, xData, yData):
        self.cleanData()
        ti = rospy.Time.now().to_sec()

        self.stopSim = 0
        rate = rospy.Rate(10000) # Hz
        t0 = rospy.Time.now().to_sec()
        dt = 1/10000
        d = 0.1
        integral = 0

        # Constantes de control
        ki = 0.1
        kp = 1
        k_angular = 6

        offset = 0.14

        for i in range(len(xData)):
            while not self.stopSim and not rospy.is_shutdown():
                self.xData.append(self.x)
                self.yData.append(self.y)
                self.timeData.append(rospy.Time.now().to_sec()-ti)
                self.thetaData.append(self.theta)

                distance = abs(math.sqrt(((xData[i]-self.x) ** 2) + ((yData[i]-self.y) ** 2)))           

                error = distance-d
                self.errorData.append(error)

                t1 = rospy.Time.now().to_sec()
                dt = t1-t0
                t0 = t1

                integral = integral + ki*error*dt

                linear_speed = offset + kp*error + integral
                self.velocityData.append(linear_speed)
                
                desired_angle_goal = math.atan2(yData[i]-self.y, xData[i]-self.x)
                gamma = self.angdiff(self.theta, desired_angle_goal)*k_angular

                self.velocity_message.linear.x = linear_speed
                self.velocity_message.linear.z = gamma

                self.pub.publish(self.velocity_message)

                if (distance<0.1):
                    self.velocity_message.linear.x = 0       
                    self.pub.publish(self.velocity_message)
                    break

                rate.sleep()

        self.velocity_message.linear.x = 0       
        self.pub.publish(self.velocity_message)
        self.saveToFile('follow_path')
        
        
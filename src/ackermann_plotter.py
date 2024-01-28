#!/usr/bin/env python3

import numpy as np
import tkinter as tk
import matplotlib as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg,
    NavigationToolbar2Tk
)

import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry

plt.use('TkAgg')

class Ackermann_Plotter:
    """
    Grafica los datos obtenidos del los scripts de control y modelo.
    - Posición del robot en el plano xy
    """
    def __init__(self):
        # ROS
        rospy.init_node('ackermann_plotter_node', anonymous=True)
        pose_subscriber = rospy.Subscriber('/odom', Odometry, self.poseCallback)

        # Variables de posición
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.xData = []
        self.yData = []

        # El robot se representa como un triángulo
        height = 0.1    # Eje vertical   (y)
        width = 0.25    # Eje horizontal (x)
        self.ack_coords_x = [-height/2, height/2, 0]
        self.ack_coords_y = [-width/2,-width/2, width/2]

        self.root = tk.Tk()

        # Matplotlib graph
        # Create a figure
        figure = Figure(figsize=(6.5, 5), dpi=150)
        self.figure_canvas = FigureCanvasTkAgg(figure, master=self.root)
        self.figure_canvas.get_tk_widget().grid(
            column=0,
            row=0,
            padx=5,
            pady=5,
            columnspan=5
        )

        # Create axes
        self.axes = figure.add_subplot()

        self.axes.set_title('Position')

        self.btnClean = tk.Button(
                self.root, 
                text = "Limpiar gráfico", 
                width = 50,
                command=self.clearPlot
            )
        self.btnClean.grid(
                column=0,
                row=1,
                padx=5,
                pady=5,
                columnspan=2
            )

        self.xpositionLabel = tk.Label(
            self.root,
            text='x = 0.0 m',
        )
        self.xpositionLabel.grid(
            column=2,
            row=1,
            padx=5,
            pady=5
        )

        self.ypositionLabel = tk.Label(
            self.root,
            text='y = 0.0 m'
        )
        self.ypositionLabel.grid(
            column=3,
            row=1,
            padx=5,
            pady=5
        )

        self.thetaLabel = tk.Label(
            self.root,
            text='theta = 0.0 rad'
        )
        self.thetaLabel.grid(
            column=4,
            row=1,
            padx=5,
            pady=5
        )
        # Función sin()
        self.path1x = np.arange(0.2, 3.0, 0.1)
        self.path1y = np.sin(self.path1x*2.0-0.8)+1

        # Señal cuadrada
        #self.path1y = np.array([0.5, 1.5, 1.5, 0.5, 0.5, 0.5, 1.5, 1.5])
        #self.path1x = np.array([0.5, 0.5, 1.5, 1.5, 2.5, 2.5, 2.5, 3.0])

        # Actualiza el gráfico
        self.update_plot()

        self.update_labels()

        self.root.mainloop()

    def clearPlot(self):
        self.yData.clear()
        self.xData.clear()
        self.axes.clear()

    def poseCallback(self, pose_message):
        """
        Recibe la odometría del robot, x, y, theta

        Args:
            pose_message (Odometry): Posición y orientación del robot
        """
        self.x= pose_message.pose.pose.position.x
        self.y= pose_message.pose.pose.position.y
        self.theta = pose_message.pose.pose.orientation.z
    
    def update_labels(self):
        self.xpositionLabel.config(text='x = {:.3} m'.format(self.x))
        self.ypositionLabel.config(text='y = {:.3} m'.format(self.y))
        self.thetaLabel.config(text='theta = {:.3} rad'.format(self.theta))

        self.root.after(200, self.update_labels)

    def update_plot(self):
        """
        Actualiza el gráfico para mostrar la pocisión y orientación del robot.
        Esta función se ejecuta cada 200 ms
        """
        # Ackermann vehicle coords
        # Rotación respecto al origen
        xAck, yAck = self.rotatez(self.ack_coords_x, self.ack_coords_y, self.theta - np.pi/2)
        # Traslación
        xAck = np.add(xAck, self.x)
        yAck = np.add(yAck, self.y)

        self.axes.clear()
        self.yData.append(self.y)
        self.xData.append(self.x)
        self.axes.plot(self.xData, self.yData, color = 'b')
        self.axes.plot(self.path1x, self.path1y, linestyle=':', color = 'r')

        self.axes.fill(xAck, yAck, 'm')
        self.axes.set_xlabel('x (m)')
        self.axes.set_ylabel('y (m)')
        self.axes.set_xlim((-0.2, 3))
        self.axes.set_ylim((-0.2, 2))
        self.axes.set_aspect('equal')
        self.axes.grid(True)

        self.axes.set_title('Position')
        self.figure_canvas.draw()
        self.root.after(200, self.update_plot)
    
    def rotatez(self, x, y, theta):
        """
        Aplica una matriz de rotación en el eje z a un par coodenado xy
        con un ángulo theta.

        Args:
            x (float): Coordenada x
            y (float): Coordenada y
            theta (float): Ángulo a rotar sobre el eje z
        Return:
            x, y (float): Nuevas coordenadas con rotación.
        """
        M = np.array([  [np.cos(theta), -np.sin(theta)],
                        [np.sin(theta),  np.cos(theta)]])
        x, y = np.dot(M, np.array([x, y]))
        return x, y

if __name__ == '__main__':
    try:
        ack_plt = Ackermann_Plotter()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")
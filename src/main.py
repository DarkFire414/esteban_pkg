#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from ackermann_control import Ackermann_Control
from ackermann_plotter import Ackermann_Plotter
import threading
import sys
import numpy as np

publiser = rospy.Publisher('/set_position', Odometry, queue_size=10)
position_msg = Odometry()

def control_fcn(control_obj, action, xd, yd, thetad):
    print("\t\t ...")
    if action == 1:
        control_obj.go_to_goal(xd, yd)
    
    elif action == 2:
        control_obj.go_to_pose(xd, yd, thetad)
    
    elif action == 3:
        control_obj.follow_path(xd, yd)

if __name__ == '__main__':
    isSimulation = 0
    args = sys.argv

    if len(args) >= 2:
        isSimulation = int(args[1])
    
        # Condiciones iniciales
        myControl = Ackermann_Control(0, 0, 0)

        thread = threading.Thread(target=control_fcn, args=(myControl,))

        if isSimulation:
            print("\n\t Esteban <Programa principal> <Simulación>")
        else:
            print("\n\t Esteban <Programa principal> <Implementación>")

        print("\n\t ¿Qué desea hacer con Esteban")
        print("\t\t a) Detener a Esteban")
        print("\t\t b) Mover a Esteban a un punto")
        print("\t\t c) Mover a Esteban a una pose")
        print("\t\t d) Seguir una trayectoria con Esteban")

        if isSimulation:
            print("\t\t e) Establecer condiciones iniciales")
            
        print("\t\t f) Salir")

        while 1:
            res = input("\n\t Ingrese la opción: ")

            if res == 'a':
                myControl.stopSim = 1

            elif res == 'b':
                xd = float(input("\t\t xd = "))
                yd = float(input("\t\t yd = "))
                thread = threading.Thread(target=control_fcn, args=(myControl, 1, xd, yd, 0))
                thread.start()
            
            elif res == 'c':
                xd = float(input("\t\t xd = "))
                yd = float(input("\t\t yd = "))
                thetad = float(input("\t\t thetad (grad) = "))*3.1416/180.0
                thread = threading.Thread(target=control_fcn, args=(myControl, 2, xd, yd, thetad))
                thread.start()
            
            elif res == 'd':
                path1x = np.arange(0.2, 3.0, 0.1)
                path1y = np.sin(path1x*2.0-0.8)+1
                thread = threading.Thread(target=control_fcn, args=(myControl, 3, path1x, path1y, 0))
                thread.start()
            
            elif res == 'e':
                position_msg.pose.pose.position.x = float(input("\t\t xd = "))
                position_msg.pose.pose.position.y = float(input("\t\t yd = "))
                position_msg.pose.pose.orientation.z = float(input("\t\t thetad (grad) = "))*3.1416/180.0
                publiser.publish(position_msg)

            elif res == 'f':
                break

        # Ejecutar la función de forma normal
        if thread.is_alive():
            thread.join()

        print("Programa terminado...")
    
    else:
        print("El programa requiere del argumento is Simulation [0, 1]")


            
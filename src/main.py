#!/usr/bin/env python3

import rospy
import pathlib
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import quaternion_from_euler
from control.stanley_controller.stanley_controller import Stanley_Controller
import threading
import sys
import time 

publiser = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

def control_fcn(control_obj, file_name = None):
    print("\t\t ...")
    control_obj.run(file_name)

def main(args):
    rospy.init_node('path_tracking_node')

    isSimulation = int(rospy.get_param('/is_simulation'))
    stanley_gains = rospy.get_param('/gains')
    wheel_base = rospy.get_param('/robot/WB')
    rate = rospy.get_param('/stanley/rate')
    
    myControl = Stanley_Controller(stanley_gains, wheel_base, rate)

    thread = threading.Thread(target=control_fcn, args=(myControl,))

    if isSimulation:
        print("\n Esteban <Programa principal> <Simulación>")
    else:
        print("\n Esteban <Programa principal> <Implementación>")

    print("\n ¿Qué desea hacer con Esteban")
    print("\t a) Detener a Esteban")

    if isSimulation:
        print("\t b) Establecer condiciones iniciales")
        
    print("\t c) Seguir trayectoria stanley (Usando RVIZ)")
    print("\t d) Seguir trayectoria stanley (Usando Coordenadas)")
    print("\t e) Seguir trayectoria stanley (Usando RVIZ) [Guardar resultados]")
    print("\t f) Seguir trayectoria stanley (Usando Coordenadas) [Guardar resultados]")
    print("\t g) Pruebas iteradas de simulación")
            
    print("\t q) Salir")

    while True:
        res = input("\n\t Ingrese la opción: ")

        if res == 'a':
            myControl.stop_robot = 1

        elif res == 'b':
            position_msg = PoseWithCovarianceStamped()
            position_msg.header.frame_id = 'odom'
            position_msg.pose.pose.position.x = float(input("\t\t x0 = "))
            position_msg.pose.pose.position.y = float(input("\t\t y0 = "))

            theta = float(input("\t\t thetad (grad) = "))*3.1416/180.0
            quaternion = quaternion_from_euler(0, 0, theta)
            position_msg.pose.pose.orientation.x = quaternion[0]
            position_msg.pose.pose.orientation.y = quaternion[1]
            position_msg.pose.pose.orientation.z = quaternion[2]
            position_msg.pose.pose.orientation.w = quaternion[3]

            publiser.publish(position_msg)

        elif res == 'c':
            if thread.is_alive():
                thread.join()
                
            thread = threading.Thread(target=control_fcn, args=(myControl,))
            thread.start()
            
        elif res == 'd':
            if thread.is_alive():
                thread.join()

            thread = threading.Thread(target=control_fcn, args=(myControl,))
            thread.start()

            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'odom'
            goal_msg.pose.position.x = float(input("\t\t x0 = "))
            goal_msg.pose.position.y = float(input("\t\t y0 = "))
            goal_publisher.publish(goal_msg)
            
        elif res == 'e':
            file_name = str(input("\t Nombre del archivo sin extensión: "))
            file_path = str(pathlib.Path(__file__).parent) + "/control/stanley_controller/saved_data/" + file_name
            
            if thread.is_alive():
                thread.join()

            thread = threading.Thread(target=control_fcn, args=(myControl, file_path))
            thread.start()
            
        elif res == 'f':
            file_name = str(input("\t Nombre del archivo sin extensión: "))
            file_path = str(pathlib.Path(__file__).parent) + "/control/stanley_controller/saved_data/" + file_name

            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'odom'
            goal_msg.pose.position.x = float(input("\t\t x0 = "))
            goal_msg.pose.position.y = float(input("\t\t y0 = "))
            goal_publisher.publish(goal_msg)

            if thread.is_alive():
                thread.join()

            thread = threading.Thread(target=control_fcn, args=(myControl, file_path))
            thread.start()
        
        elif res == 'g':
            position_msg = PoseWithCovarianceStamped()
            position_msg.header.frame_id = 'odom'
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'odom'            
            iterations_number = int(input("\t\t Número de iteraciones = "))
            theta = float(input("\t\t thetad (grad) = "))*3.1416/180.0
            #se almacena automaticamente la ruta generada
            file_name = str(input("\t Nombre del archivo sin extensión: "))
            file_path = str(pathlib.Path(__file__).parent) + "/path_planning/path_saved_data/" + file_name
            print(file_path)
            for i in range(iterations_number):
                #publicacion de la posicion incial deseada
                position_msg.pose.pose.position.x = 0.2
                position_msg.pose.pose.position.y = 0.2                                     
                quaternion = quaternion_from_euler(0, 0, theta)
                position_msg.pose.pose.orientation.x = quaternion[0]
                position_msg.pose.pose.orientation.y = quaternion[1]
                position_msg.pose.pose.orientation.z = quaternion[2]
                position_msg.pose.pose.orientation.w = quaternion[3]
                publiser.publish(position_msg)
                time.sleep(1)
                #publicacion del punto final deseado
                goal_msg.pose.position.x = 0.9
                goal_msg.pose.position.y = 1.5
                goal_publisher.publish(goal_msg)
                

            if thread.is_alive():
                thread.join()
            thread = threading.Thread(target=control_fcn, args=(myControl, file_path))
            thread.start()



        elif res == 'q':
            myControl.stop_robot = 1
            break

    # Terminar hilo
    if thread.is_alive():
        thread.join()

    print("\t Programa terminado...")

if __name__ == '__main__':
    main(sys.argv)


            
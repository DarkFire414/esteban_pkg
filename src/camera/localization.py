#!/usr/bin/env python3

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

import pickle

import cv2
from cv2 import aruco
import numpy as np

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from utils.image_converter import Image_converter

def get_mark_tvec(num_id, ids, rvecs, tvecs):
    ### If marker of num_id is detected ###
    if num_id in np.ravel(ids) :
        index = np.where(ids == num_id)[0][0] #Extract index of num_id
        tvec = tvecs[index][0]
        return tvec
    return None

def get_relative_coordinates3(aruco_id_1, aruco_id_2, ids, rvecs, tvecs):
    if aruco_id_1 in np.ravel(ids) and aruco_id_2 in np.ravel(ids):
        index_1 = np.where(ids == aruco_id_1)[0][0]
        index_2 = np.where(ids == aruco_id_2)[0][0]
        rvec_1 = rvecs[index_1]
        tvec_1 = tvecs[index_1]
        rvec_2 = rvecs[index_2]
        tvec_2 = tvecs[index_2]

        # Convertir rvec y tvec a matrices de transformación
        R_1, _ = cv2.Rodrigues(rvec_1)
        T_1 = np.hstack((R_1, tvec_1.reshape(3, 1)))
        T_1 = np.vstack((T_1, [0, 0, 0, 1]))  # Agregar fila [0, 0, 0, 1]

        R_2, _ = cv2.Rodrigues(rvec_2)
        T_2 = np.hstack((R_2, tvec_2.reshape(3, 1)))
        T_2 = np.vstack((T_2, [0, 0, 0, 1]))  # Agregar fila [0, 0, 0, 1]

        # Calcular la matriz de transformación relativa de aruco_id_1 a aruco_id_2
        T_relative = np.linalg.inv(T_1) @ T_2

        # Obtener la matriz de rotación relativa
        R_relative = T_relative[:3, :3]

        # Calcular el ángulo de rotación en grados
        angle_radians = np.arctan2(R_relative[1, 0], R_relative[0, 0])
        #angle_degrees = np.degrees(angle_radians)

        # Obtener las coordenadas (x, y) del centro del ArUco 2 en el sistema de coordenadas del ArUco 1
        x = T_relative[0, 3]
        y = T_relative[1, 3]

        return x, y, angle_radians

    return None

class Aruco_camera:
    def __init__(self, 
                aruco_origin,
                aruco_robot, 
                marker_length, 
                calibration_file_path,
                window_size,
                rate
                ):
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.sub_image = rospy.Subscriber('/camera_image', Image, self.image_callback)

        self.rate = rospy.Rate(rate)

        self.odom_msg = Odometry()

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
    
        # Obtener las matrices de calibración
        calibration = None
        try:
            with open(calibration_file_path, 'rb') as archivo:
                calibration = pickle.load(archivo)
        except Exception as e:
            print("LOCALIZATION (ERROR): " + e.message)
    
        self.cameraMatrix = calibration[0]
        self.distCoeffs = calibration[1]

        self.image_cv2 = None

        self.img_converter = Image_converter(callback_fcn=None)

        self.arucoOrigin = aruco_origin
        self.arucoRobot = aruco_robot
        self.marker_length = marker_length

        self.ventana = []
        self.ventana_size = window_size
    
    def image_callback(self, msg):
        self.image_cv2 = self.img_converter.rosimg_to_cv2(msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.image_cv2 is not None:
                # Convierte el fotograma a escala de grises
                gray = cv2.cvtColor(self.image_cv2, cv2.COLOR_BGR2GRAY)
                # Detecta los marcadores Aruco en el fotograma
                corners, ids, rejected = aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.parameters
                    )
                
                if ids is not None:
                    # Dibuja los contornos y los identificadores de los marcadores detectados
                    image_cv2 = aruco.drawDetectedMarkers(self.image_cv2, corners, ids)

                    #Calcula las coordenadas 3D de los marcadores si se detectaron
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.cameraMatrix, self.distCoeffs)
        
                    if (self.arucoOrigin in ids) and (self.arucoRobot in ids):
                        # Si encuentra los arucos del origen y el robot
                        tvecOrigin = get_mark_tvec(self.arucoOrigin, ids, rvecs, tvecs)
                        tvecRobot = get_mark_tvec(self.arucoRobot,  ids, rvecs, tvecs)

                        if (tvecOrigin is not None) and (tvecRobot is not None):
                            # Distancia euclidiana
                            #distance = np.linalg.norm(tvecOrigin-tvecRobot)

                            # Se ontienen las coordenadas relativas al aruco origen
                            x, y, angle = get_relative_coordinates3(self.arucoOrigin, self.arucoRobot, ids, rvecs, tvecs)
                            #print("x = {:.3f}, y = {:.3f},angulo= {:.3f}".format(x, y, angle))
                            #print("distancia = {:f}".format(distance))

                            # Insertamos las coordenadas obtenidas en la ventana de filtrado
                            self.ventana.append((x, y, angle))
                            # Mantiene el tamaño de la ventana acotado y actualiza las coordenadas dentro de la ventana
                            if len(self.ventana) > self.ventana_size: 
                                self.ventana.pop(0)
                                #print("x = {:.3f}, y = {:.3f},angulo= {:.3f}".format(x, y, angle))
                                #print("distancia = {:f}".format(distance))
                            #Aplicamos el filtro promedio movil
                            promedio_x = sum(coord[0] for coord in self.ventana) / len(self.ventana)
                            promedio_y = sum(coord[1] for coord in self.ventana) / len(self.ventana)
                            promedio_angle = sum(coord[2] for coord in self.ventana) / len(self.ventana)

                            self.odom_msg.pose.pose.position.x = promedio_x
                            self.odom_msg.pose.pose.position.y = promedio_y
                            self.odom_msg.pose.pose.orientation.z = promedio_angle

                            self.pub_odom.publish(self.odom_msg)
                
                #self.rate.sleep()

                # Muestra el fotograma procesado
                cv2.imshow('Aruco Camera',image_cv2)

                # Detiene el bucle si se presiona la tecla 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        cv2.destroyAllWindows()

def main(args):
    rospy.init_node('ackermann_camera_node')

    arucoOrigin = int(rospy.get_param('/camera/aruco_origin'))
    arucoRobot =  int(rospy.get_param('/camera/aruco_robot'))
    marker_length = float(rospy.get_param('/camera/marker_length')) 
    calibrationFilePath = rospy.get_param('~calibrationFilePath')
    window_size = int(rospy.get_param('/camera/window_size'))
    rate = int(rospy.get_param('/camera/rate'))

    localizator = Aruco_camera(
        aruco_origin=arucoOrigin,
        aruco_robot=arucoRobot,
        marker_length=marker_length,
        calibration_file_path=calibrationFilePath,
        window_size=window_size,
        rate=rate
    )

    print("LOCALIZATION: Running...")
    localizator.run()

if __name__ == '__main__':
    main(sys.argv)

    


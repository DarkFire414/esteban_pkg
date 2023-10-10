#!/usr/bin/env python3

import pickle

import cv2
from cv2 import aruco
import numpy as np

import rospy
from nav_msgs.msg import Odometry

"""
cameraMatrix = np.array ([[497.58172002   ,0.     ,    311.91187996]
 ,[  0.     ,    496.907022   ,226.58519438]
 ,[  0.     ,      0.      ,     1.        ]])
distCoeffs = np.array ([[ 1.98865419e-01 ,-8.86778326e-01 , 6.11165447e-04  ,1.67759951e-04,1.24455843e+00]])
"""
'''
distCoeffs = np.array ([[ 8.09263819e-02, -3.22193799e-01, -4.92832623e-05, -8.29849053e-04, 4.60139445e-01]])
cameraMatrix = np.array([[455.95461573,   0.,         317.60530093],
                         [  0.,         455.81727576, 242.42659694],
                         [  0.,           0.,           1.        ]])
'''
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

if __name__ == '__main__':
    rospy.init_node('ackermann_camera_node')

    pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    rate = rospy.Rate(2500) # Hz
    last_time = rospy.Time.now()

    cmd = Odometry()

    cameraIp = rospy.get_param('~cameraIp')
    arucoOrigin = int(rospy.get_param('~arucoOrigin'))
    arucoRobot =  int(rospy.get_param('~arucoRobot'))
    marker_length = float(rospy.get_param('~marker_length'))  # Longitud del marcador Aruco en metros
    calibrationFilePath = rospy.get_param('~calibrationFilePath')

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    # Obtener las matrices de calibración
    with open(calibrationFilePath, 'rb') as archivo:
        calibration = pickle.load(archivo)
    
    cameraMatrix = calibration[0]
    distCoeffs = calibration[1]

    cap = cv2.VideoCapture(cameraIp) 

    while True:
        # Captura un fotograma de la cámara
        ret, frame = cap.read()

        # Convierte el fotograma a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detecta los marcadores Aruco en el fotograma
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        if ids is not None:
            # Dibuja los contornos y los identificadores de los marcadores detectados
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

            # Calcula las coordenadas 3D de los marcadores si se detectaron
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, cameraMatrix, distCoeffs)

            if (arucoOrigin in ids) and (arucoRobot in ids):
                # Si encuentra los arucos del origen y el robot
                tvecOrigin = get_mark_tvec(arucoOrigin, ids, rvecs, tvecs)
                tvecRobot = get_mark_tvec(arucoRobot,  ids, rvecs, tvecs)

                if (tvecOrigin is not None) and (tvecRobot is not None):
                    # Distancia euclidiana
                    #distance = np.linalg.norm(tvecOrigin-tvecRobot)

                    # Se ontienen las coordenadas relativas al aruco origen
                    x, y, angle = get_relative_coordinates3(arucoOrigin, arucoRobot, ids, rvecs, tvecs)
                    #print("x = {:.3f}, y = {:.3f},angulo= {:.3f}".format(x, y, angle))
                    #print("distancia = {:f}".format(distance))

                    cmd.pose.pose.position.x = x
                    cmd.pose.pose.position.y = y
                    cmd.pose.pose.orientation.z = angle

                    pub.publish(cmd)
        
        rate.sleep()

        # Muestra el fotograma procesado
        cv2.imshow('Frame', frame)

        # Detiene el bucle si se presiona la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Libera los recursos y cierra las ventanas
    cap.release()
    cv2.destroyAllWindows()


#!/usr/bin/env python3

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

import pickle

import cv2
from cv2 import aruco
import numpy as np

import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from utils.image_converter import Image_converter

def censurar_imagen(frame, ancho_centro, alto_centro):
    # Obtener las dimensiones del frame
    alto, ancho = frame.shape[:2]

    # Crear una máscara completamente negra del mismo tamaño que el frame
    mascara = np.zeros((alto, ancho), dtype=np.uint8)

    # Calcular las coordenadas del centro del frame
    centro_x = ancho // 2
    centro_y = alto // 2

    # Calcular los límites del área a no censurar
    x1 = max(0, centro_x - ancho_centro // 2)
    y1 = max(0, centro_y - alto_centro // 2)
    x2 = min(ancho, centro_x + ancho_centro // 2)
    y2 = min(alto, centro_y + alto_centro // 2)

    # Establecer el área no censurada en blanco en la máscara
    mascara[y1:y2, x1:x2] = 255

    # Aplicar la máscara al frame para censurar el resto de la imagen
    frame_censurado = cv2.bitwise_and(frame, frame, mask=mascara)

    return frame_censurado

#acomodar frame
def ordenar_puntos(puntos):
    n_puntos = np.concatenate([puntos[0], puntos[1], puntos[2], puntos[3]]).tolist()
    y_order = sorted(n_puntos, key=lambda n_puntos: n_puntos[1])
    x1_order = y_order[:2]
    x1_order = sorted(x1_order, key=lambda x1_order: x1_order[0])
    x2_order = y_order[2:4]
    x2_order = sorted(x2_order, key=lambda x2_order: x2_order[0])
    
    return [x1_order[0], x1_order[1], x2_order[0], x2_order[1]]

def roi(image, ancho, alto):
    imagen_alineada = None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, th = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
    #comprueba el ubral bro acuerdate 
    
    #cv2.imshow('recortado', th) 
    
    #_, cnts, _  = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #cnts, _  = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    _, cnts, _  = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:1]
   
    #comprueba el contornote bro acuerdate
    """
    contour_image = image.copy()
    cv2.drawContours(contour_image, cnts, -1, (0, 255, 0), 2)
    cv2.imshow('recortado', contour_image)  
    """
    for c in cnts:
        # Aproximar la forma del contorno a un polígono
        approx = cv2.approxPolyDP(c, 0.04 * cv2.arcLength(c, True), True)
        #epsilon = 0.01*cv2.arcLength(c,True)
        #approx = cv2.approxPolyDP(c,epsilon,True)
        #print(len(approx))
        if len(approx) == 4: #4
            puntos = ordenar_puntos(approx)            
            pts1 = np.float32(puntos)
            pts2 = np.float32([[0,0], [ancho,0], [0,alto], [ancho,alto]])
            M = cv2.getPerspectiveTransform(pts1, pts2)
            imagen_alineada = cv2.warpPerspective(image, M, (ancho,alto))
        else:
            #print('Se han detectado ' + str(len(approx)) + 'lados')
            imagen_alineada = image.copy()
    return imagen_alineada

def detect_green_objects(image,pixel_cm_ratio):
        # Convertir la imagen a espacio de color HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Definir el rango de color verde en HSV
    lower_green = np.array([100, 50, 50])
    upper_green = np.array([130, 255, 255])
    
    # Crear una máscara para el rango de color verde
    mask = cv2.inRange(hsv, lower_green, upper_green)
    # Aplicar un filtro para excluir píxeles oscuros (valor < 50)
    mask = cv2.bitwise_and(mask, cv2.inRange(hsv[...,2], 70, 255))
    #cv2.imshow('Genera Mapa', mask)
    # Find contours
    # _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
    # encuentra contornos en el frame de obstaculos

    return cnts

class Genera_Mapa:
    def __init__(self,
                aruco_origin,
                marker_length,
                calibration_file_path,
                rate
                ):

        self.sub_image = rospy.Subscriber('/camera_image', Image, self.image_callback)
        self.distancias_pub = rospy.Publisher('distancias_topic', String, queue_size=100)
        
        self.rate = rospy.Rate(rate)
        
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
        self.arucoOrigin = aruco_origin
        self.marker_length = marker_length

        self.datos_circulos = []

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
    
    def image_callback(self, msg):
        self.image_cv2 = self.img_converter.rosimg_to_cv2(msg)
    
    def distancias(self, marker_center, pixel_cm_ratio):
        mensajes = []  # Lista para almacenar todos los mensajes
            # Calcula la distancia (x, y) entre el primer rectángulo y los demás
        obstaculos = "" 
        for i, circulo in enumerate(self.datos_circulos):  
            x, y, _ = circulo
            dx = (abs(x) - abs(marker_center[0]))/pixel_cm_ratio
            dy = -(abs(y) - abs(marker_center[1]))/pixel_cm_ratio
            distancia = np.sqrt(dx**2 + dy**2)  # Calcula la distancia usando el teorema de Pitágoras
            object_diam = circulo[2]
            #print(f"Circulo {i}: Distancia (x={dx}, y={dy}), diametro={object_diam}")
            #Crear cadena con la informacion de cada obstaculo
            obstaculos = obstaculos + f"Circulo {i}: Distancia (x={dy}, y={dx}), diametro={object_diam}"
            # Crear un mensaje de tipo Vector3Stamped y almacenarlo en la lista
            '''mensaje = Vector3Stamped()
            mensaje.vector.x = dx
            mensaje.vector.y = dy
            mensaje.vector.z = object_diam # Opcional: puedes usar z para otro propósito si lo necesitas
            mensajes.append(mensaje)'''
        #Publicar el string con todos los obstaculos
        self.distancias_pub.publish(obstaculos)
        self.rate.sleep()
        # Publicar todos los mensajes almacenados en la lista
        '''for mensaje in mensajes:
            distancias_pub.publish(mensaje)'''
        #distancias_pub.flush()
        #rate.sleep()
    
    def run(self):
        while not rospy.is_shutdown():
            if self.image_cv2 is not None:
                # Si la lectura fue exitos
                frame_ajustado = cv2.undistort(self.image_cv2, self.cameraMatrix, self.distCoeffs)
                """
                    183/152 = 1.2039
                    alto=720/1.2039
                    alto 598   
                """
                frame_ajustado = censurar_imagen(frame_ajustado, 600, 520)
                
                frame_ajustado = roi(frame_ajustado,598, 720)
                #cv2.imshow('recortado', frame_ajustado) 

                if frame_ajustado is not None and  np.any(frame_ajustado):
                    # Convierte el fotograma a escala de grises
                    gray = cv2.cvtColor(frame_ajustado, cv2.COLOR_BGR2GRAY)

                    # Detecta los marcadores Aruco en el fotograma
                    corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
                    # Aplica la corrección de distorsión a la imagen

                    if ids is not None:
                        #calculo rvecs y tvecs
                        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.cameraMatrix, self.distCoeffs)
                        # Aruco Perimeter
                        aruco_perimeter = cv2.arcLength(corners[0], True)
                        # Pixel to cm ratio
                        pixel_cm_ratio = aruco_perimeter / 60
                        #print(pixel_cm_ratio)
                        contornos = detect_green_objects(frame_ajustado,pixel_cm_ratio)
                        self.datos_circulos.clear()
                        j = 0
                        for cnt in contornos:
                            # Get circulos
                            # Encuentra el círculo mínimo que encierra el contorno
                            (x, y), radius = cv2.minEnclosingCircle(cnt)
                            # Calcula el diámetro del círculo
                            object_diameter = radius * 2 / pixel_cm_ratio
                            # Dibuja el círculo
                            center = (int(x), int(y))
                            radius = int(radius)
                            if radius > 4:
                                cv2.circle(frame_ajustado, center, radius, (255, 0, 0), 2)
                                    # Dibuja un punto rojo en el centro del círculo
                                cv2.circle(frame_ajustado, center, 3, (0, 0, 255), -1)
                                # Muestra el diámetro
                                    # Muestra el diámetro y el número de círculo
                                cv2.putText(frame_ajustado, f"{j}", 
                                            (int(x - 30), int(y + 50)), cv2.FONT_HERSHEY_PLAIN, 2, (100, 200, 0), 2)
                                cv2.putText(frame_ajustado, "{}".format(round(object_diameter, 1)), 
                                            (int(x ), int(y +50)), cv2.FONT_HERSHEY_PLAIN, 2, (100, 200, 0), 2)
                                j = j +1
                                # Guarda los datos del círculo
                                self.datos_circulos.append((x, y, object_diameter))
                        marker_id_to_find = self.arucoOrigin
                        if ids is not None and marker_id_to_find in ids:
                            # encontrar marcador centro 
                            
                            marker_index = np.where(ids == marker_id_to_find)[0][0]
                            marker_corners = corners[marker_index][0]
                            marker_center = np.mean(marker_corners, axis=0)
                            # Dibujar el centro del marcador
                            marker_center = tuple(map(int, marker_center))
                            cv2.circle(frame_ajustado, marker_center, 5, (0, 255, 0), -1)
                            # Dibuja los contornos y los identificadores de los marcadores detectados
                            frame_ajustado = aruco.drawDetectedMarkers(frame_ajustado, corners, ids)
                            self.distancias(marker_center,pixel_cm_ratio)
                    
                    # Muestra el fotograma procesado
                    
                    cv2.imshow('Genera Mapa', frame_ajustado)
                    # Si se presiona 's' (código de tecla 115), guarda el frame como imagen y calcula las distancias
                    if cv2.waitKey(1) & 0xFF == ord('s'):
                        cv2.imwrite("frame_capturado.png", frame_ajustado)
                    # Detiene el bucle si se presiona la tecla 'q'
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

        cv2.destroyAllWindows()

def main(args):
    rospy.init_node('distancias_publisher')

    arucoOrigin = int(rospy.get_param('/camera/aruco_origin'))
    marker_length = float(rospy.get_param('/camera/marker_length'))  # Longitud del marcador Aruco en metros
    calibrationFilePath = rospy.get_param('~calibrationFilePath')
    rate = int(rospy.get_param('/costmap_obstacles/rate'))

    obstacle_detector = Genera_Mapa(
        aruco_origin=arucoOrigin,
        marker_length=marker_length,
        calibration_file_path=calibrationFilePath,
        rate=rate
    )

    print("OBS DETECTOR: Running...")
    obstacle_detector.run()

if __name__ == '__main__':
    main(sys.argv)

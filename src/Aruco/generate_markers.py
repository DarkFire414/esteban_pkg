#!/usr/bin/env python3

import cv2
import numpy as np
import cv2.aruco as aruco

# Carga el diccionario de arucos
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50) #size is 4 by 4 and number is 50 - faster

def aruco_create(path):
    """
    Genera los marcadores Aruco como imagen y los guarda en una carpeta.

    Args:

        path (string): Ruta Completa a la carpeta donde se guardaran los 
                        marcadores
    """
    for i in range(50):
        outputMarker = np.zeros((200, 200), dtype=np.uint8)
        markerImage = cv2.aruco.drawMarker(dictionary, i, 500, outputMarker, 1) #500 is the number of pixels
        image_path = path + '/mark_id_{}.jpg'.format(i)
        cv2.imwrite(image_path , markerImage)
        print(image_path)

if __name__ == '__main__':
    path = '/home/angel/catkin_ws/src/esteban_pkg/src/Aruco/markers'
    print("Generando marcadores")
    aruco_create(path)
    print("...")
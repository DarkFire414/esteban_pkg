"""
Este script toma una serie de fotos de un patrón de 
calibracióon (pattern.png) se debe mostrar este patrón 
impreso o en imagen frente a la cámara y presionar la tecla s 
para guardar la imagen.
"""

import cv2

# Usar la cámara a calibrar
cap = cv2.VideoCapture('http://192.168.0.102:4747/video')

num = 0

print('\nPresione la tecla s para capturar la imagen esc para terminar...')

while cap.isOpened():
    succes, img = cap.read()
    k = cv2.waitKey(5)

    if k == 27:
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite('/home/javier/catkin_ws/src/esteban_pkg_tests/src/Aruco/calibration_images/img' + str(num) + '.png', img)
        print("{} image saved!".format(num))
        num += 1

    cv2.imshow('Img',img)

# Release and destroy all windows before termination
cap.release()

cv2.destroyAllWindows()

"""
Imprime en consola las matrices de calibración y de distorción
obtenidas con el script calibration.py
"""
import pickle

with open('/home/angel/catkin_ws/src/esteban_pkg/src/Aruco/calibration_results/calibration.pkl', 'rb') as archivo:
    calibration = pickle.load(archivo)

with open('/home/angel/catkin_ws/src/esteban_pkg/src/Aruco/calibration_results/cameraMatrix.pkl', 'rb') as archivo:
    cameraMatrix = pickle.load(archivo)

with open('/home/angel/catkin_ws/src/esteban_pkg/src/Aruco/calibration_results/dist.pkl', 'rb') as archivo:
    dist = pickle.load(archivo)

print("\n calibration.pkl")
print(calibration[0])
print(calibration[1])
print("\n cameraMatrix.pkl")
print(cameraMatrix)
print("\n dist.pkl")
print(dist)
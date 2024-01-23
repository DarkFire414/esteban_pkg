import pathlib

PI = 3.14159265359

class C:
    """
    Clase con los parámetros principales para la simulación
    """
    k = 10.0                 # [s^-1] Control gain
    kv = 1                  # [m/s]  Stanley Velocity gain

    kp = 5.0                # Speed proportional gain
    ki = 0.3                # Speed integrative gain
    kd = 0.05               # Speed derivative gain
    dt = 0.1                # [s] tiempo entre cada actualización

    # Características físicas
    RF = 0.180              # [m] Distancia desde la parte trasera hasta la parte delantera
    RB = 0.065              # [m] Distancia desde el eje trasero hasta la parte trasera del vehículo
    W  = 0.145              # [m] Ancho del vehículo
    WD = 0.119              # [m] Distancia entre las llantas 
    WB = 0.137              # [m] Distancia entre las llantas delantera y trasera, de centro a centro L
    TR = 0.031              # [m] Radio de las llantas
    TW = 0.026              # [m] Ancho de las llantas
    MAX_STEER = PI/6.0   # [rad] Ángulo máximo que puede girar el steering

    path = str(pathlib.Path(__file__).parent)
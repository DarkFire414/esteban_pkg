import numpy as np
import math

class Ackermann_robot:
    """
    Modelo cinemático de un robot tipo Ackermann
    """
    def __init__(self, x0=0.0, y0=0.0, theta0=0.0, v0=0.0, C=None):
        """
        Args:
            x (float): Posición en x del robot (Medido desde el centro) [m]
            y (float): Posición en y del robot (Medido desde el centro) [m]
            theta (float): Ángulo de rotación del robot con respecto al eje Z
                medido desde el eje x [rad]
            v (float): Velocidad lineal [m/s]
        """
        # Posición del robot medido desde el centro de las ruedas traseras
        self.x = x0
        self.y = y0
        self.theta = theta0
        self.v = v0
        self.gamma = 0
        self.C = C

    def updateWithCameraData(self, v, gamma, theta, x, y):
        """
        Actualiza la posición del vehículo con base en la informacion de la camara

        Args:
            acc (float): Aceleración del robot
            gamma (float): Ángulo del eje delantero
            theta (float): Ángulo de orientacion del carrito
            x: Posicion X del carrito
            y: Posicion Y del carrito
        """
        # Saturación del ángulo gamma (steering)
        self.gamma = np.clip(gamma, -self.C.MAX_STEER, self.C.MAX_STEER)

        self.x = x
        self.y = y
        self.theta = theta
        self.v = v

        # El ángulo theta debe estar normalizado entre valores de -pi a pi
        self.theta = normalize_angle(self.theta)

    def update(self, acc, gamma):
        """
        Actualiza la posición del vehículo con base en el modelo cinemático

        Args:
            acc (float): Aceleración del robot
            gamma (float): Ángulo del eje delantero
        """
        # Saturación del ángulo gamma (steering)
        self.gamma = np.clip(gamma, -self.C.MAX_STEER, self.C.MAX_STEER)

        self.x += self.v * math.cos(self.theta) * self.C.dt
        self.y += self.v * math.sin(self.theta) * self.C.dt
        self.theta += (self.v / self.C.WB) * math.tan(self.gamma) * self.C.dt
        self.v += acc * self.C.dt

        # El ángulo theta debe estar normalizado entre valores de -pi a pi
        self.theta = normalize_angle(self.theta)

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.theta = []
        self.v = []
        self.t = []
        self.gamma = []

        self.cross_track_error = []
    
    def addState(self, t, ackermann_state, efa):
        self.x.append(ackermann_state.x)
        self.y.append(ackermann_state.y)
        self.theta.append(ackermann_state.theta)
        self.v.append(ackermann_state.v)
        self.t.append(t)
        self.gamma.append(ackermann_state.gamma)

        self.cross_track_error.append(efa)
    
    def save_to_txt(self, file_name):
        with open(file_name, 'w') as file:
            # Escribir encabezado
            file.write("t\tx(t)\ty(t)\ttheta(t)\tv(t)\tgamma(t)\tefa\n")

            # Escribir datos
            for i in range(len(self.t)):
                timei = self.t[i]
                xi = self.x[i]
                yi = self.y[i]
                thetai = self.theta[i]
                vi = self.v[i]
                gammai = self.gamma[i]
                efai = self.cross_track_error[i]

                # Escribir una línea de datos en el archivo
                file.write("{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\n".format(timei, xi, yi, thetai, vi, gammai, efai))

import numpy as np
from path_planning.cubic_spline_planner import cubic_spline_planner

class Trajectory:
    def __init__(self):
        self.px = []
        self.py = []
        self.ptheta = []
    
    def generatePathViap(self, px, py):
        self.px = []
        self.py = []
        self.ptheta = []
        self.px, self.py, _cyaw, _ck, _s = cubic_spline_planner.calc_spline_course(px, py, ds=0.001)
        # Obtener la orientación de la trayectoria usando la primera derivada
        dx = np.gradient(self.px)
        dy = np.gradient(self.py)
        self.ptheta = np.arctan2(dy, dx)
    
    def generatePath(self, px, py):
        self.px = px
        self.py = py
        self.ptheta = []
        # Obtener la orientación de la trayectoria usando la primera derivada
        dx = np.gradient(self.px)
        dy = np.gradient(self.py)
        self.ptheta = np.arctan2(dy, dx)
    
    def load_from_txt(self, fileName):
        self.px = []
        self.py = []
        self.ptheta = []
        # Leer el archivo
        with open(fileName, 'r') as archivo:
            # Leer cada línea del archivo
            for linea in archivo:
                # Separar las coordenadas usando '\t' (tabulador) como delimitador
                coordenadas = linea.strip().split('\t')
                
                # Convertir las coordenadas a números y agregar a los arreglos x e y
                self.px.append(float(coordenadas[0]) * 0.05)
                self.py.append(float(coordenadas[1]) * 0.05)

        # Obtener la orientación de la trayectoria usando la primera derivada
        dx = np.gradient(self.px)
        dy = np.gradient(self.py)
        self.ptheta = np.arctan2(dy, dx)

class Ackermann_States:
    def __init__(self):
        """
        Almacena la información del robot Ackermann para cada instante de tiempo
        """
        self.x = []
        self.y = []
        self.theta = []
        self.x_path = []
        self.y_path = []
        self.theta_path = []

        self.v = []
        self.t = []
        self.gamma = []

        self.cross_track_error = []
    
    def set_path(self, xp, yp, thetap):
        """
        Guarda las coordenadas de la ruta

        Args:
            xp (float array): Coordenadas x [m]
            yp (float array): Coordenadas y [m]
            thetap (float array): Ángulos [rad]
        """
        self.x_path = xp.copy()
        self.y_path = yp.copy()
        self.theta_path = thetap.copy()
    
    def addState(self, t, x, y, theta, v, gamma, efa):
        """
        Agrega un nuevo estado

        Args:
            t (float): Instante de tiempo [s]
            x (float): Posición en x(t) [m]
            y (float): Posición en y(t) [m]
            theta (float): Ángulo (yaw) [rad]
            v (float): Velocidad v(t) [m/s]
            gamma (float): Ańgulo de dirección [rad]
            efa (float): Error lateral [m]
        """
        self.x.append(x)
        self.y.append(y)
        self.theta.append(theta)
        self.v.append(v)
        self.t.append(t)
        self.gamma.append(gamma)
        self.cross_track_error.append(efa)
    
    def save_data_to_txt(self, file_name):
        """
        Guarda los estados en un archivo txt con el formato:
            t x(t) y(t) ...
            ...
            ...
        Args:
            file_name (string): Ruta del archivo a guardar <sin extensión> ex: /home/src/control/resultados
        """
        with open(file_name + "_data.txt", 'w') as file:
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
                file.write("{:.6f}\t{:.6f}\t{:.6f}\t{:.6f}\t{:.6f}\t{:.6f}\t{:.6f}\n".format(timei, xi, yi, thetai, vi, gammai, efai))

    def save_path_to_txt(self, file_name):
        """
        Guarda las coordenadas de la ruta en un archivo txt con el formato:
            t x(t) y(t) ...
            ...
            ...
        La ruta y los pasos llevados a cabo por el robot no necesariamente tienen el mismo 
        tamaño.
        
        Args:
            file_name (string): Ruta del archivo a guardar <sin extensión> ex: /home/src/control/resultados
        """
        with open(file_name + "_path.txt", 'w') as file:
            # Escribir encabezado
            file.write("x(i)\ty(i)\ttheta(i)\n")

            # Escribir datos
            for i in range(len(self.x_path)):
                xpi = self.x_path[i]
                ypi = self.y_path[i]
                thetapi = self.theta_path[i]

                # Escribir una línea de datos en el archivo
                file.write("{:.6f}\t{:.6f}\t{:.6f}\n".format(xpi, ypi, thetapi))

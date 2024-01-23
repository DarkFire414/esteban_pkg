import numpy as np

class Control:
    def __init__(self, C):
        self.prev_error = 0
        self.integral = 0
        self.C = C

    def stanley_control(self, state, cx, cy, cyaw, last_target_idx):
        """
        Control de seuigmiento de trayectoria Stanley.

        Args:
            state (object Ackermann_robot)
            cx (float array): Puntos en x de la trayectoria [m]
            cy (float array): Puntos en y de la trayectoria [m]
            cyaw (float array): Orientación para cada punto en la trayectoria [rad]
            last_target_idx (int): último índice al que el vehículo se dirigió
        
        Returns:
            delta (float): Steering angle
            current_target_idx (int): índice del punto de la trajectoria
            error_fron_axle (float): error lateral
        """
        # Obtiene el índice el punto al cuál dirigirse y el error lateral
        current_target_idx, error_front_axle = self.calc_target_index(state, cx, cy)
        
        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = normalize_angle(cyaw[current_target_idx] - state.theta)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(self.C.k * error_front_axle, self.C.kv + state.v)

        # Steering control
        delta = theta_e + theta_d

        return delta, current_target_idx, error_front_axle
    
    def proportional_control(self, target, current):
        """
            Control proporcional
        """
        acc = self.C.Kp * (target - current)
        return acc
    
    def pid_control(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.C.kp * error + self.C.ki * self.integral + self.C.kd * derivative

        self.prev_error = error
        return output
    
    def calc_target_index(self, state, cx, cy):
        """
        Obtiene el índice sobre la trayectoria al cuál el 
        vehículo debe dirigirse

        Args:
            state: (object Ackermann_robot)
            cx (float array): Puntos en x de la trayectoria [m]
            cy (float array): Puntos en y de la trayectoria [m]
        
        Return:
            target_idx (int): ínice objetivo
            error_front_axl (float): Error lateral
        """
        # Calc front axle position
        fx = state.x + self.C.WB * np.cos(state.theta)
        fy = state.y + self.C.WB * np.sin(state.theta)

        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)                               

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(state.theta + np.pi / 2),
                          -np.sin(state.theta + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle

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
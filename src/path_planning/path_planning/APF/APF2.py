import numpy as np
import matplotlib.pyplot as plt
import random
import math
from suaviza.mientras_llueve import approximate_b_spline_path

def add_goal(X, Y, s, r, loc, x, y):
    delx = np.zeros_like(X)
    dely = np.zeros_like(Y)
    for i in range(len(x)):
        for j in range(len(y)):
            d = np.sqrt((loc[0] - X[i][j]) ** 2 + (loc[1] - Y[i][j]) ** 2)
            # print(f"{i} and {j}")
            theta = np.arctan2(loc[1] - Y[i][j], loc[0] - X[i][j])
            if d < r:
                delx[i][j] = 0
                dely[i][j] = 0
            elif d > r + s:
                delx[i][j] = 50 * s * np.cos(theta)
                dely[i][j] = 50 * s * np.sin(theta)
            else:
                delx[i][j] = 50 * (d - r) * np.cos(theta)
                dely[i][j] = 50 * (d - r) * np.sin(theta)
    return delx, dely

def add_obstacle(X, Y, s, r, delx, dely, goal, x, y, coords):
    obstacle = coords
    for i in range(len(x)):
        for j in range(len(y)):
            d_goal = np.sqrt((goal[0] - X[i][j]) ** 2 + ((goal[1] - Y[i][j])) ** 2)
            d_obstacle = np.sqrt((obstacle[0] - X[i][j]) ** 2 + (obstacle[1] - Y[i][j]) ** 2)
            # print(f"{i} and {j}")
            theta_goal = np.arctan2(goal[1] - Y[i][j], goal[0] - X[i][j])
            theta_obstacle = np.arctan2(obstacle[1] - Y[i][j], obstacle[0] - X[i][j])
            if d_obstacle < r:
                delx[i][j] = -1 * np.sign(np.cos(theta_obstacle)) * 5 + 0
                dely[i][j] = -1 * np.sign(np.cos(theta_obstacle)) * 5 + 0
            elif d_obstacle > r + s:
                delx[i][j] += 0 - (50 * s * np.cos(theta_goal))
                dely[i][j] += 0 - (50 * s * np.sin(theta_goal))
            elif d_obstacle < r + s:
                delx[i][j] += -150 * (s + r - d_obstacle) * np.cos(theta_obstacle)
                dely[i][j] += -150 * (s + r - d_obstacle) * np.sin(theta_obstacle)
            if d_goal < r + s:
                if delx[i][j] != 0:
                    delx[i][j] += (50 * (d_goal - r) * np.cos(theta_goal))
                    dely[i][j] += (50 * (d_goal - r) * np.sin(theta_goal))
                else:
                    delx[i][j] = (50 * (d_goal - r) * np.cos(theta_goal))
                    dely[i][j] = (50 * (d_goal - r) * np.sin(theta_goal))
            if d_goal > r + s:
                if delx[i][j] != 0:
                    delx[i][j] += 50 * s * np.cos(theta_goal)
                    dely[i][j] += 50 * s * np.sin(theta_goal)
                else:
                    delx[i][j] = 50 * s * np.cos(theta_goal)
                    dely[i][j] = 50 * s * np.sin(theta_goal)
            if d_goal < r:
                delx[i][j] = 0
                dely[i][j] = 0
    return delx, dely, obstacle, r

def smooth_path(path):
    if path:
        n_course_point = 200
        x, y = zip(*path)
        # bspline aproximation
        rax, ray, heading, curvature = approximate_b_spline_path(
            y, x, n_course_point, s=0.5)
        return list(zip(ray, rax, heading))
    else:
        return False

def plot_graph(X, Y, delx, dely, obj, fig, ax, loc, r, i, color, start_goal=np.array([[0, 0]])):
    ax.quiver(X, Y, delx, dely)
    ax.add_patch(plt.Circle(loc, r, color=color))
    ax.set_title(f'Robot path with {i} obstacles ')
    ax.annotate(obj, xy=loc, fontsize=10, ha="center")
    return ax
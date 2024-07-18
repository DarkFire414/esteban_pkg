## Trabajo terminal: Desarrollo de un robot autónomo

*`Esteban`* es un robot móvil autónomo tipo [`Ackermann`](https://en.wikipedia.org/wiki/Ackermann_steering_geometry), específicamente el [`PICAR-V`](https://docs.sunfounder.com/projects/picar-v/en/latest/introduction.html) de la marca Sunfounder, programado en [`ROS (Robot Operating System)`](https://www.ros.org/) con el objetivo de comparar algunos algoritmos de planeación de trayectoria determinísticos (A*, Campos Potenciales Artificiales y Dijkstra) considerando las métricas de tiempo de cómputo, longitud del camino generado y el error de seguimiento de trayectoria al tratar de seguir dicho camino. Para el seguimiento de trayectoria se implementó el controlador [`Stanley`](http://robotics.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf) de la universidad de Stanford.

## Funcionamiento

En el siguiente enlace de Youtube, se muestra un video demostrativo de cada algoritmo evaluado (Dijkstra, A* y Campos Potenciales)

https://youtu.be/aA__u1j6Fbw?si=lvCmOyOpBwgdVoeG

![Obstacle avoidance](resources/images/obstacle-avoidance.gif)
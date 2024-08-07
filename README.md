## Trabajo terminal: Desarrollo de un robot autónomo

*`Esteban`* es un robot móvil autónomo tipo [`Ackermann`](https://en.wikipedia.org/wiki/Ackermann_steering_geometry), específicamente el [`PICAR-V`](https://docs.sunfounder.com/projects/picar-v/en/latest/introduction.html) de la marca Sunfounder, programado en [`ROS (Robot Operating System)`](https://www.ros.org/) con el objetivo de comparar algunos algoritmos de planeación de trayectoria determinísticos (A*, Campos Potenciales Artificiales y Dijkstra) considerando las métricas de tiempo de cómputo, longitud del camino generado y el error de seguimiento de trayectoria al tratar de seguir dicho camino. Para el seguimiento de trayectoria se implementó el controlador [`Stanley`](http://robotics.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf) de la universidad de Stanford.

## Funcionamiento

En el siguiente [Video demostrativo](https://www.youtube.com/watch?v=aA__u1j6Fbw) de Youtube, se muestra un video demostrativo de cada algoritmo evaluado (Dijkstra, A* y Campos Potenciales)

[![Video demostrativo](https://img.youtube.com/vi/aA__u1j6Fbw/0.jpg)](https://www.youtube.com/watch?v=aA__u1j6Fbw)

Funcioamiento en físico.
![Obstacle avoidance](resources/images/obstacle-avoidance.gif)

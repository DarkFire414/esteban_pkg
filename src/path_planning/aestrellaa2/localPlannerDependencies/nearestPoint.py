import math

def point_distance(punto1, punto2):
    return math.sqrt((punto2[0] - punto1[0]) ** 2 + (punto2[1] - punto1[1]) ** 2)

def found_nearest_point(punto_dado, arreglo):
    distancia_minima = float('inf')
    punto_mas_cercano = None

    for punto_en_arreglo in arreglo:
        distancia = point_distance(punto_dado, punto_en_arreglo)
        if distancia < distancia_minima:
            distancia_minima = distancia
            punto_mas_cercano = punto_en_arreglo

    return punto_mas_cercano
import numpy as np
import heapq
import random
import math
from HybridAstar.HybridAstar import AstarAckermann

def localPathGenerator(previous_path, start2, goal, grid2):
    astar_local_instance = AstarAckermann(start2, goal, grid2)
    NewRawAstarPath = previous_path[:-1] + astar_local_instance.astar()
    if (NewRawAstarPath):
        return NewRawAstarPath, astar_local_instance.smoother(NewRawAstarPath)
    else:
        False
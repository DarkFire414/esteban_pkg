import numpy as np

def inflate_map(grid, inflation_distance):
    rows, cols = grid.shape
    inflated_map = np.copy(grid)
    for i in range(rows):
        for j in range(cols):
            if grid[i, j] == 1:
                for ni in range(i - inflation_distance, i + inflation_distance + 1):
                    for nj in range(j - inflation_distance, j + inflation_distance + 1):
                        if 0 <= ni < rows and 0 <= nj < cols:
                            inflated_map[ni, nj] = 1
    return inflated_map
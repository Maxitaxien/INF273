from pathlib import Path
import numpy as np

def load_data(path: Path):
    with path.open(mode='r', encoding='utf-8') as f:
        # Number of customers
        f.readline()
        n = int(f.readline().strip())

        # Range limit
        f.readline()
        lim = int(f.readline().strip())

        # Travel time matrix for truck
        f.readline()
        truck_matrix = []
        for _ in range(n + 1):
            truck_matrix.append(np.array(list(map(float, f.readline().strip().split()))))

        # Travel time matrix for drone
        f.readline()
        drone_matrix = []
        for _ in range(n + 1):
            drone_matrix.append(list(map(float, f.readline().strip().split())))

    return (n, lim, np.array(truck_matrix), np.array(drone_matrix))
import numpy as np


class MAPF:
    def __init__(self) -> None:
        self.eps = 5
        self.eta = 200
        self.rho = 5  # rep distance
        self.l = 4

    def gravitational_force(self, drones):
        f = np.zeros([len(drones), 3])
        return self.eps * (drones.target - drones.position)

    def repulsive_force(self, drones):
        f = np.zeros([len(drones), 3])
        for i in range(len(drones)):
            tmp =  drones.position[i] - drones.position + 1e-9
            f = self.eta * (1 / tmp - 1 / self.eps) * ( 1 / tmp ** 2) * (tmp / np.linalg.norm(tmp))
            f[np.linalg.norm(tmp,axis=1) < self.rho] = 0
        return np.sum(f, axis = 1)

    def next_step(self, drones):
        gf = self.gravitational_force(drones)
        rf = self.repulsive_force(drones)
        fsum = gf + rf
        drones.position += fsum / np.linalg.norm(fsum) * self.l

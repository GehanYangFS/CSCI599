import numpy as np


class MAPF:
    def __init__(self) -> None:
        self.eps = 5
        self.eta = 200
        self.rho = 0.5  # rep distance
        self.l = 4

    def gravitational_force(self, drones):
        f = np.zeros([len(drones), 3])
        return self.eps * (drones.target - drones.position)

    def repulsive_force(self, drones):
        f = np.zeros([len(drones), 3])
        for i in range(len(drones)):
            tmp =  drones.position[i] - drones.position
            tmp[tmp < 1e-3] = 0
            tmp2 = self.eta * (1 / tmp - 1 / self.rho) * ( 1 / tmp ** 2) * (tmp / np.linalg.norm(tmp))
            tmp2[np.isnan(tmp2)] = 0
            tmp2[(np.linalg.norm(tmp,axis=1) > self.rho)] = 0
            f[i] = np.sum(tmp2, axis = 0)
        return f

    def next_step(self, drones):
        gf = self.gravitational_force(drones)
        rf = self.repulsive_force(drones)
        fsum = gf + rf
        newPosition = fsum / np.linalg.norm(fsum) * self.l
        drones.position[np.linalg.norm(drones.position-drones.target,axis=1)>=1]+= newPosition[np.linalg.norm(drones.position-drones.target,axis=1)>=1]

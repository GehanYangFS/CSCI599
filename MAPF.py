import numpy as np
from utils import normalize
import copy

from droneconfig import Drone, MultiDrones, Dispathcers
import pdb
class MAPF:
    def __init__(self) -> None:
        self.eps = 5
        self.eta = 400
        self.rho = 1.8  # rep distance
        self.l = 1
        self.target_eps = 0.2
        self.cache = [0,0]
        self.STEPS_PER_DISPATCH = 1
        self.EXIT_THRESHOLD = 1
    
    def add_deployment(self, deployment):
        self.deployment = Dispathcers(deployment)
        self.allowed = set()

    def gravitational_force(self, drones):
        f = np.zeros([len(drones), 3])
        return self.eps * (drones.target - drones.position)

    def repulsive_force(self, drones):
        # pdb.set_trace()
        f = np.zeros([len(drones), 3])
        for i in range(len(drones)):
            tmp =  drones.position[i] - drones.position
            tmp2 = self.eta * (1 / tmp - 1 / self.rho) * ( 1 / tmp ** 2) * (tmp / np.linalg.norm(tmp))
            tmp2[np.isnan(tmp2)] = 0
            tmp2[np.abs(tmp) < 1e-5] = 0
            tmp2[(np.linalg.norm(tmp,axis=1) >= self.rho)] = 0
            # tmp[:, 2] = -tmp[:, 2]
            tmp2 = np.abs(tmp2) * np.sign(tmp)
            f[i] = np.sum(tmp2, axis = 0)
            # print(tmp2)
        return f
    
    def dynamic_constraint(self, drones):
        dc = np.zeros([len(drones),3])
        for i in range(len(drones)):
            tmp = np.linalg.norm(drones.position[i] - drones.position, axis= 1)
            tmp[i] = np.max(tmp)
            tmp2 = np.linalg.norm(drones.position[i] - drones.target, axis= 1)
            dc[i] = min(np.min(tmp),np.min(tmp2))
        return dc

    def next_step(self, drones, step):
        gf = self.gravitational_force(drones)
        rf = self.repulsive_force(drones)
        self.cache[0] = gf
        self.cache[1] = rf
        # print(rf[0], gf[0])
        # input('.')
        # gf[rf > 0] = 0
        fsum = gf + rf
        # print(gf)
        # print(rf)
        # print(fsum)
        # pdb.set_trace()
        # l = self.dynamic_constraint(drones) / 4
        # l[l > self.l] = self.l
        l = np.ones_like(fsum) * self.l
        newPosition = fsum / np.linalg.norm(fsum) * l
        # if np.sum(np.linalg.norm(newPosition,axis=1)>1) > 1:
        #     pdb.set_trace()
        drones.preposition = copy.deepcopy(drones.position)
        drones.position[np.linalg.norm(drones.position-drones.target,axis=1)>=self.target_eps]+= newPosition[np.linalg.norm(drones.position-drones.target,axis=1)>=self.target_eps]
        if np.sum(np.linalg.norm(drones.position-drones.target,axis=1)>=self.target_eps) <= self.EXIT_THRESHOLD:
            return 0
        if step % self.STEPS_PER_DISPATCH == 0:
            self.allowed = self.deployment.allow_dispathed
        if len(self.deployment.all) != len(self.allowed):
            for idx in self.deployment.all-self.allowed:
                drones.position[idx] = copy.deepcopy(drones.preposition[idx])
            
        if np.sum(np.linalg.norm(drones.position-drones.target,axis=1)>=self.target_eps) == 0:
            print('all done')
            return 0
        # self.TargetExchange(drones)

    def TargetExchange(self, drones):
        for d in range(len(drones)):
            case1 = 0 
            case1TF = False
            case2TF = False
            case3TF = False
            tmp3 = []
            tmp4 = []
            for i in range(len(drones)):
                if i == d:
                    continue
                if((np.linalg.norm(drones.position[i] - drones.position[d]) < self.rho) and np.linalg.norm(drones.position[i] - drones.target[i]) < self.target_eps):
                    case1 = case1 + 1
                    tmp3.append(i)
                if case1 >= 2:
                    case1TF = True
            if(case1TF):
                for i in range(len(tmp3)):
                    if (np.linalg.norm(drones.position[tmp3[i]] - drones.target[d])) < np.linalg.norm((drones.position[d] - drones.target[d])):
                        tmp4.append(i)
                        case2TF = True
                if(case2TF):
                    if (np.linalg.norm(drones.preposition[d] - drones.target[d])) <(np.linalg.norm(drones.position[d] - drones.target[d])):
                        case3TF = True
            if(case3TF):
                tmpmin  = np.linalg.norm(drones.position[tmp4[0]] - drones.position[d])
                tmpr = tmp4[0]
                for r in range(1,len(tmp4)):
                    if (np.linalg.norm(drones.position[r] - drones.position[d]) < tmpmin):
                        tmpmin = np.linalg.norm(drones.position[r] - drones.position[d])
                        tmpr = r
                print('触发！')
                drones.target[d] , drones.target[tmpr] = drones.target[tmpr], drones.target[d]
            # print(case1TF, case2TF, case3TF)

        
if __name__ == '__main__':
    mapf = MAPF()
    rf = mapf.repulsive_force(
        MultiDrones([
            Drone(1,1,np.array([4,4,1]),None),
            Drone(1,1,np.array([4,4,2]),None)
            ]
        )
        )
    print(rf)
    rf = mapf.repulsive_force(
        MultiDrones([
            Drone(1,1,np.array([4,4,-2.2]),None),
            Drone(1,1,np.array([4,4,-2.1]),None),
            Drone(1,1,np.array([4,4,-2]),None)
            ]
        )
    )
    print(rf)

    rf = mapf.repulsive_force(
        MultiDrones([
            Drone(1,1,np.array([4,4,-3]),None),
            Drone(1,1,np.array([4,4,-2.5]),None),
            Drone(1,1,np.array([4,4,-2]),None)
            ]
        )
    )
    print(rf)
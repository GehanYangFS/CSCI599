import math
import numpy as np
from droneconfig import Drone

class FailureHandling:
    Y_OFFSET = 10
    Z_OFFSET = 2
    SIM_LIGHT_FAILURES = 5

    @classmethod
    def handle_deployment(cls, deployment:  dict[int, list[int]], overhead: float) -> dict[int, list[int]]:
        """_summary_

        Args:
            deployment (dict[int, list[int]]): the deloyment of drones sorted by descending 
            distance for each dispatcher
            overhead (float): percentage of extra drones for failure handling

        Returns:
            dict[int, list[int]]: the deloyment of drones with addition of failure handling
            deploymnet
        """
        dispatcher_idx: int = max(list(deployment.keys())) + 1
        pose_idx: int = max([_v for v in deployment.values() for _v in v]) + 1
        n_extra = math.ceil(pose_idx * overhead)
        deployment[dispatcher_idx] = []
        for i in range(n_extra):
            deployment[dispatcher_idx].append(pose_idx)
            pose_idx += 1
        return deployment

    @classmethod
    def handle_poses(cls, poses: list[list[int]], overhead: float):
        """_summary_

        Args:
            poses (list[list[int]]): target poses of drones

        Returns:
            list[list[int]]: target poses of drones with addition of backup
            drones
        """
        n_extra = math.ceil(len(poses) * overhead)
        print(n_extra)
        maxy = np.max(np.array(poses)[:,1]) + cls.Y_OFFSET
        maxz = np.max(np.array(poses)[:,2])
        maxx = np.max(np.array(poses)[:,0])
        pose_ids = []
        for i in range(n_extra):
            poses.append([maxx, maxy, maxz - i * cls.Z_OFFSET])
            pose_ids.append(len(poses)-1)
        return poses, pose_ids

    @classmethod
    def sim_light_failures(cls, drone: Drone):
        drone.light = False
    
    @classmethod
    def sim_rotor_failures(cls, drone: Drone):
        drone.rotor = False
from turtle import pos
import airsim
from PointCloud import PointCloud
from Config import Config
import time
import numpy as np


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        norm = np.finfo(v.dtype).eps
    return v/norm


class Control:
    def __init__(self, cfg: Config, step_size: float) -> None:
        self.cfg = cfg
        self.step_size = step_size

        # connect to the AirSim simulato
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

        self.num_drones = len(cfg.droneNames)
        self._enableApiControl()

    def _enableApiControl(self):
        for i in range(self.num_drones):
            vehicle_name = self.cfg.droneNames[self.num_drones-1-i]
            self.client.enableApiControl(True, vehicle_name)
            self.client.armDisarm(True, vehicle_name)

    def step(self, poses):
        for i in range(numDrones):
            vehicle_name = cfg.droneNames[numDrones-1-i]
            pose = self.client.simGetVehiclePose(vehicle_name=vehicle_name)
            cpose = np.array(
                [pose.position.x_val, pose.position.y_val, pose.position.z_val])
            vpose = normalize(np.array(poses[i]) - cpose)
            pose.position = pose.position + \
                airsim.Vector3r(*(vpose * self.step_size))
            # pose.orientation = pose.orientation + airsim.to_quaternion(0.0, 0.0, 0.0)
            self.client.simSetVehiclePose(
                pose, True, vehicle_name=vehicle_name)


if __name__ == '__main__':
    cfg = Config('baseSettings.json',
                 'C:/Users/shenb/Documents/AirSim/settings.json')
    cfg.generateDrones(
        [[0, 0, 0], [4, 0, 0], [4, 4, 0], [0, 4, 0]],
        100//4
    )
    input('Press any key to continue...')

    numDrones = len(cfg.droneNames)
    poses = PointCloud("", numDrones).getPoints()

    main_control = Control(cfg, 1)

    for _ in range(100):
        main_control.step(poses)

import airsim
from algorithms import MinDist
from pointcloud import PointCloud, AlphabetPointCloud
from droneconfig import Config
import time
import numpy as np
import os
from flags import Flag_ue_executable_file, Flag_ue_executable_settings_path
from algorithms import MinDist, QuotaBalanced



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
        for dispathcer, drone_names in self.cfg.dispatchers.items():
            for drone in drone_names[::-1]:
                self.client.enableApiControl(True, drone)
                self.client.armDisarm(True, drone)

    def step(self, poses):
        for i in range(numDrones):
            vehicle_name = cfg.droneNames[numDrones-1-i].name
            pose = self.client.simGetVehiclePose(vehicle_name=vehicle_name)
            cpose = np.array(
                [pose.position.x_val, pose.position.y_val, pose.position.z_val], dtype=np.float32)
            vpose = normalize(np.array(poses[i]) - cpose)
            pose.position = pose.position + \
                airsim.Vector3r(*(vpose * self.step_size))
            # pose.orientation = pose.orientation + airsim.to_quaternion(0.0, 0.0, 0.0)
            self.client.simSetVehiclePose(
                pose, True, vehicle_name=vehicle_name)

    def _moveToPosition(self, target_pose, vehicle_name, dispatcher):
        pose = self.client.simGetVehiclePose(vehicle_name=vehicle_name)
        pose.position = airsim.Vector3r(*target_pose) - airsim.Vector3r(*dispatcher)
        self.client.simSetVehiclePose(
            pose, True, vehicle_name=vehicle_name)
        time.sleep(0.1)

    def moveToPosition(self, dispatchers, poses):
        for dispathcer, drone_names in self.cfg.dispatchers.items():
            for drone in drone_names[::-1]:
                self._moveToPosition(poses[cfg.droneNames[drone].pose_id], drone, dispatchers[dispathcer])
            

if __name__ == '__main__':
    cfg = Config('baseSettings.json',
                 Flag_ue_executable_settings_path)

    poses = []
    apc = AlphabetPointCloud(downwash=1)
    poses.extend(apc.query_point_cloud(alphabet = 'U', volume= 3))
    poses.extend(apc.query_point_cloud(alphabet = 'S', offset = [0, 20, 0], volume= 3))
    poses.extend(apc.query_point_cloud(alphabet = 'C', offset = [0, 40, 0], volume= 3))
    numDrones = len(poses)
    dispatchers = [
        [0,0,0], [0,4,0], [4,0,0], [4,4,0]
    ]
    deployment = QuotaBalanced(poses, dispatchers, [numDrones // len(dispatchers) + 1 for _ in range(len(dispatchers))])

    cfg.generateDrones(
        dispatchers,
        numDrones,
        deployment
    )

    for drone in cfg.droneNames:
        cfg.droneNames[drone].target = poses[cfg.droneNames[drone].pose_id]
    # os.startfile(Flag_ue_executable_file)
    # input('Press any key to continue...')

    main_control = Control(cfg, 1)

    main_control.moveToPosition(dispatchers, poses)
    # for _ in range(100):
    #     main_control.step(poses)

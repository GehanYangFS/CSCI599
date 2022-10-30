import imp
from multiprocessing.spawn import old_main_modules
import airsim
from algorithms import MinDist
from pointcloud import PointCloud, AlphabetPointCloud
from droneconfig import Config, MultiDrones
import time
import numpy as np
import os
from flags import Flag_ue_executable_file, Flag_ue_executable_settings_path
from algorithms import MinDist, QuotaBalanced
from MAPF import MAPF
import cProfile
import pdb
import copy
from utils import normalize
from tqdm import tqdm

class Control:
    def __init__(self, cfg: Config, step_size: float) -> None:
        self.cfg = cfg
        self.step_size = step_size

        # connect to the AirSim simulato
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

        self.num_drones = len(cfg.droneNames)
        self._enableApiControl()
        
        self.mapf = []

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
        pose.position = airsim.Vector3r(*target_pose) - airsim.Vector3r(*self.cfg.droneNames[vehicle_name].position)
        self.client.simSetVehiclePose(
            pose, True, vehicle_name=vehicle_name)

    def moveToPosition(self, dispatchers, poses):
        for dispathcer, drone_names in self.cfg.dispatchers.items():
            for drone in drone_names[::-1]:
                self._moveToPosition(poses[self.cfg.droneNames[drone].pose_id], drone, dispatchers[dispathcer])
        self.detect_collision()
    
    def detect_collision(self):
        for drone in self.cfg.all_drones:
            coll = self.client.simGetCollisionInfo(vehicle_name=drone.name)
            if coll.has_collided == True:
                continue
                pdb.set_trace()
                print(coll.object_name)
                print(coll.position)
                pose = self.client.simGetVehiclePose(coll.object_name)
                pose.position += airsim.Vector3r(*self.cfg.droneNames[coll.object_name].position)
                pose.position.z_val -= -1
                self.client.simSpawnObject('11', 'PointLightBP',pose ,airsim.Vector3r(1,1,1))
                print('gf:', self.mapf[0].cache[0][self.cfg.droneNames[coll.object_name].pose_id])
                print('rf:', self.mapf[0].cache[1][self.cfg.droneNames[coll.object_name].pose_id])
                
def TargetExchangeFailCaseTest():
    dispatchers = [
        [0,0,-1], [0,4,-1], [4,0,-1], [4,4,-1]
    ]
    destination = [
        [4,4, -1], [4,0,-1], [0,4,-1], [0,0,-1]
    ]
    cfg = Config('baseSettings.json',
                 Flag_ue_executable_settings_path)

    cfg.generateDrones(
        dispatchers,
        1,
        {}
    )
    idx = 0
    for drone in cfg.droneNames:
        cfg.droneNames[drone].target = destination[idx]
        idx += 1
    input('Press any key to continue...')
    main_control = Control(cfg, 1)

    # main_control.moveToPosition(dispatchers, poses)

    mapf = MAPF()
    main_control.mapf.append(mapf)
    mdrones = MultiDrones(main_control.cfg.all_drones)
    for i in range(1000000):
        code = mapf.next_step(mdrones)
        if code == 0:
            break
        main_control.moveToPosition(dispatchers, mdrones.position)
        for drone in main_control.cfg.all_drones:
            pos = main_control.client.simGetVehiclePose(drone.name).position
            mdrones.position[drone.pose_id] = np.array([pos.x_val, pos.y_val, pos.z_val]) + drone.position

        # if i % 1000 == 0:
            # input('...')


def TargetExchangeFailCaseTest2():
    dispatchers = [
        [0,0,-1]
    ]
    destination = [
        [0,0, -10], [0,0,-9], [0,0,-8], [0,0,-7]
    ]
    cfg = Config('baseSettings.json',
                 Flag_ue_executable_settings_path)

    cfg.generateDrones(
        dispatchers,
        4,
        {}
    )
    idx = 0
    for drone in cfg.droneNames:
        cfg.droneNames[drone].target = destination[idx]
        idx += 1
    input('Press any key to continue...')
    main_control = Control(cfg, 1)

    # main_control.moveToPosition(dispatchers, poses)

    mapf = MAPF()
    main_control.mapf.append(mapf)
    mdrones = MultiDrones(main_control.cfg.all_drones)
    for i in range(1000000):
        code = mapf.next_step(mdrones)
        if code == 0:
            break
        main_control.moveToPosition(dispatchers, mdrones.position)
        for drone in main_control.cfg.all_drones:
            pos = main_control.client.simGetVehiclePose(drone.name).position
            mdrones.position[drone.pose_id] = np.array([pos.x_val, pos.y_val, pos.z_val]) + drone.position

        # if i % 1000 == 0:
            # input('...')


if __name__ == '__main__':
    # TargetExchangeFailCaseTest2()
    cfg = Config('baseSettings.json',
                 Flag_ue_executable_settings_path)

    poses = []
    apc = AlphabetPointCloud(downwash=2)
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
    input('Press any key to continue...')

    main_control = Control(cfg, 1)

    # main_control.moveToPosition(dispatchers, poses)

    sim = True
    mapf = MAPF()
    main_control.mapf.append(mapf)
    mdrones = MultiDrones(main_control.cfg.all_drones)
    prev = copy.deepcopy(mdrones.position)
    for i in tqdm(range(1000000)):
        # cProfile.run('mapf.next_step(mdrones)')
        code = mapf.next_step(mdrones)
        
        for i in range(len(mdrones.position)):
            dist = np.linalg.norm(mdrones.position[i] - mdrones.position, axis = 1)
            dist[i] = max(dist)
            dist[dist>0.5] = 0
            if dist.sum() > 0:
                print('coll!')
        if code == 0:
            break
        # print(np.linalg.norm(prev-mdrones.position, axis = 1))
        # print(prev)
        # for j in range(len(prev)):
        #     if (np.linalg.norm(prev[j]-mdrones.position, axis = 1) < 1).sum() > 1:
        #         pdb.set_trace()
        # prev = copy.deepcopy(mdrones.position)
        if sim:
            main_control.moveToPosition(dispatchers, mdrones.position)
            for drone in main_control.cfg.all_drones:
                pos = main_control.client.simGetVehiclePose(drone.name).position
                mdrones.position[drone.pose_id] = np.array([pos.x_val, pos.y_val, pos.z_val]) + drone.position

        # if i % 1000 == 0:
            # input('...')

    # for _ in range(100):
    #     main_control.step(poses)

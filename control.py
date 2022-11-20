import imp
import airsim
from pointcloud import PointCloud, AlphabetPointCloud
from droneconfig import Config, MultiDrones, Drone
import time
import numpy as np
import os
from flags import Flag_ue_executable_file, Flag_ue_executable_settings_path, Flag_failures, Flag_multiframe
from algorithms import MinDist, QuotaBalanced
from failures import FailureHandling
from MAPF import MAPF
import cProfile
import pdb
import copy
from utils import normalize
from tqdm import tqdm
import random
from multiframe import Motill


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

    def _moveToPosition(self, target_pose, vehicle_name):
        pose = self.client.simGetVehiclePose(vehicle_name=vehicle_name)
        pose.position = airsim.Vector3r(
            *target_pose) - airsim.Vector3r(*self.cfg.droneNames[vehicle_name].position)
        self.client.simSetVehiclePose(
            pose, True, vehicle_name=vehicle_name)
        self.cfg.droneNames[vehicle_name].cur = target_pose.copy()

    def moveToPosition(self, poses):
        for dispathcer, drone_names in self.cfg.dispatchers.items():
            for drone in drone_names[::-1]:
                self._moveToPosition(
                    poses[self.cfg.droneNames[drone].pose_id], drone)
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
                pose.position += airsim.Vector3r(
                    *self.cfg.droneNames[coll.object_name].position)
                pose.position.z_val -= -1
                self.client.simSpawnObject(
                    '11', 'PointLightBP', pose, airsim.Vector3r(1, 1, 1))
                print('gf:', self.mapf[0].cache[0]
                      [self.cfg.droneNames[coll.object_name].pose_id])
                print('rf:', self.mapf[0].cache[1]
                      [self.cfg.droneNames[coll.object_name].pose_id])


if __name__ == '__main__':
    cfg = Config('baseSettings.json',
                 Flag_ue_executable_settings_path)

    all_poses = []
    apc = AlphabetPointCloud(downwash=2)
    frames = ['USC', 'TROJANS']
    volumes = {
        'USC': 5,
        'TROJANS': 2
    }
    offsets = {
        'USC': 20,
        'TROJANS': 20
    }
    for frame in frames:
        pose = []
        for idx, c in enumerate(frame):
            pose.extend(apc.query_point_cloud(alphabet=c, offset = [0, offsets[frame] * idx, 0], volume=volumes[frame]))
        all_poses.append(np.array(pose))
        print(len(pose))
    # raise
    
    poses = all_poses[0]
    numDrones = len(poses)
    dispatchers = [
        [0, 0, 0], [0, 4, 0], [4, 0, 0], [4, 4, 0]
    ]
    collectors = np.array([8, 8, 0], dtype=np.float32)
    deployment = QuotaBalanced(poses, dispatchers, [
                               numDrones // len(dispatchers) + 1 for _ in range(len(dispatchers))])
    if Flag_failures:
        deployment = FailureHandling.handle_deployment(deployment, 0.05)
        poses, standby_pose_ids = FailureHandling.handle_poses(poses, 0.05)

    cfg.generateDrones(
        dispatchers + [[10, 10, 0]],
        numDrones,
        deployment
    )

    for drone in cfg.droneNames:
        cfg.droneNames[drone].target = poses[cfg.droneNames[drone].pose_id]
    # os.startfile(Flag_ue_executable_file)
    input('Press any key to continue...')

    main_control = Control(cfg, 1)

    ALLOW_MAPF = False

    if ALLOW_MAPF:
        mapf = MAPF()
        mapf.add_deployment(deployment)
        main_control.mapf.append(mapf)
        mdrones = MultiDrones(main_control.cfg.all_drones)
        for i in tqdm(range(1000000)):
            code = mapf.next_step(mdrones, i)
            if code == 0:
                break
            main_control.moveToPosition(mdrones.position)
            for drone in main_control.cfg.all_drones:
                pos = main_control.client.simGetVehiclePose(
                    drone.name).position
                mdrones.position[drone.pose_id] = np.array(
                    [pos.x_val, pos.y_val, pos.z_val]) + drone.position
    else:
        main_control.moveToPosition(poses)

    if Flag_failures:
        # start light failure handling
        ALLOW_MAPF = True
        input('Press any key to start light failure handling conditions ...')
        print('Start handle light failures')
        for _ in range(FailureHandling.SIM_LIGHT_FAILURES):
            light_failed_drone : Drone = random.choice(main_control.cfg.all_drones)
            while light_failed_drone.stand_by == True or light_failed_drone.available == False:
                light_failed_drone : Drone = random.choice(main_control.cfg.all_drones)
            light_failed_drone.target = collectors
            light_failed_drone.available = False
            poses[light_failed_drone.pose_id] = collectors
            id = sorted([[np.linalg.norm(light_failed_drone.cur-main_control.cfg.all_drones[_id].cur), _id] for _id in standby_pose_ids if main_control.cfg.all_drones[_id].stand_by == True])[0][1]
            standby_drone = main_control.cfg.all_drones[id]
            standby_drone.stand_by = False
            standby_drone.target = light_failed_drone.cur
            poses[id] = light_failed_drone.cur

        if ALLOW_MAPF:
            mapf = MAPF()
            mapf.add_deployment(deployment)
            main_control.mapf.append(mapf)
            mdrones = MultiDrones(main_control.cfg.all_drones)
            for i in tqdm(range(1000000)):
                code = mapf.next_step(mdrones, i)
                if code == 0:
                    break
                main_control.moveToPosition(mdrones.position)
                for drone in main_control.cfg.all_drones:
                    pos = main_control.client.simGetVehiclePose(
                        drone.name).position
                    mdrones.position[drone.pose_id] = np.array(
                        [pos.x_val, pos.y_val, pos.z_val]) + drone.position
        else:
            main_control.moveToPosition(poses)


    if Flag_multiframe:
        input('Press any key to start multiframe transformation ...')
        print('Start multiframe transformation')
        ALLOW_MAPF = True
        new_poses = all_poses[1]
        trans, extra = Motill.simple(MultiDrones(main_control.cfg.all_drones).position, np.array(new_poses))
        for pair in trans:
            main_control.cfg.all_drones[pair[0]].target = new_poses[pair[1]]
            poses[pair[0]] = new_poses[pair[1]]
        for e in extra:
            main_control.cfg.all_drones[e].target = collectors
            poses[e] = collectors
        if ALLOW_MAPF:
            mapf = MAPF()
            mapf.add_deployment(deployment)
            main_control.mapf.append(mapf)
            mdrones = MultiDrones(main_control.cfg.all_drones)
            for i in tqdm(range(1000000)):
                code = mapf.next_step(mdrones, i)
                if code == 0:
                    break
                main_control.moveToPosition(mdrones.position)
                for drone in main_control.cfg.all_drones:
                    pos = main_control.client.simGetVehiclePose(
                        drone.name).position
                    mdrones.position[drone.pose_id] = np.array(
                        [pos.x_val, pos.y_val, pos.z_val]) + drone.position
        else:
            main_control.moveToPosition(poses)

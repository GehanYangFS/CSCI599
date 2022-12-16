
import sys
import os
# getting the name of the directory
# where the this file is present.
current = os.path.dirname(os.path.realpath(__file__))

# Getting the parent directory name
# where the current directory is present.
parent = os.path.dirname(os.path.dirname(current))

# adding the parent directory to
# the sys.path.
sys.path.append(parent)


import imp
import airsim
from pointcloud import PointCloud, AlphabetPointCloud
from droneconfig import Config, MultiDrones, Drone
import time
import numpy as np
import os
from flags import Flag_ue_executable_settings_path, Flag_failures, Flag_multiframe
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
from control import Control
from multiprocessing import Pool, cpu_count, set_start_method, freeze_support




def step(l, rep):
    print(f'{l}, {rep} has started!')
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
        all_poses.append(pose)
        print(len(pose))
    poses = [np.array([1,1,1])]
    numDrones = len(poses)
    dispatchers = [
        [100, 100, 0]
    ]
    collectors = np.array([30, 30, 0], dtype=np.float32)
    deployment = QuotaBalanced(poses, dispatchers, [
                                numDrones // len(dispatchers) + 1 for _ in range(len(dispatchers))])
    if Flag_failures:
        deployment = FailureHandling.handle_deployment(deployment, 0.05)
        poses, standby_pose_ids = FailureHandling.handle_poses(poses, 0.05)

    cfg.generateDrones(
        dispatchers + [[30, 30, 0]],
        numDrones,
        deployment
    )

    for drone in cfg.droneNames:
        cfg.droneNames[drone].target = poses[cfg.droneNames[drone].pose_id]
    # os.startfile(Flag_ue_executable_file)


    ALLOW_MAPF = True
    main_control = Control(cfg, 1, True)
    mapf = MAPF()
    mapf.l = l
    mapf.rho = rep
    mapf.add_deployment(deployment)
    main_control.mapf.append(mapf)
    mdrones = MultiDrones(main_control.cfg.all_drones)
    allowed = set()
    coll = 0
    steps = 0
    f = open('experiment/result2.txt' , 'a', encoding = 'utf-8')
    for i in tqdm(range(1000000)):
        if i > 6000:
            f.write(f'{l}, {rep}, infty, {coll}\n')
            break
        steps = i
        code = mapf.next_step(mdrones, i, True)
        if i % mapf.STEPS_PER_DISPATCH == 0:
            allowed = mapf.allowed
        if len(mapf.allowed) == 165:
            for i in range(len(mdrones.position)):
                dist = np.linalg.norm(mdrones.position[i] - mdrones.position, axis = 1)
                dist[i] = max(dist)
                dist[dist>0.5] = 0
                dist[dist>0] = 1
                # if dist.sum() > 0:
                #     print('coll!')
                coll += dist.sum()
        if code == 0:
            break
    f.write(f'{l}, {rep}, {steps}, {coll}\n')
    f.flush()
    f.close()
    print(f'{l}, {rep} has ended!')


def multi_step(l, rep):
    print(f'{l}, {rep} has started!')
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
        all_poses.append(pose)
        print(len(pose))

    poses = all_poses[0]
    numDrones = len(poses)
    dispatchers = [
        [0, 0, 0], [0, 26, 0], [16, 0, 0], [16, 26, 0]
    ]
    collectors = np.array([30, 30, 0], dtype=np.float32)
    deployment = QuotaBalanced(poses, dispatchers, [
                                numDrones // len(dispatchers) + 1 for _ in range(len(dispatchers))])
    if Flag_failures:
        deployment = FailureHandling.handle_deployment(deployment, 0.05)
        poses, standby_pose_ids = FailureHandling.handle_poses(poses, 0.05)

    cfg.generateDrones(
        dispatchers + [[30, 30, 0]],
        numDrones,
        deployment
    )

    for drone in cfg.droneNames:
        cfg.droneNames[drone].target = poses[cfg.droneNames[drone].pose_id]
        cfg.droneNames[drone].cur = poses[cfg.droneNames[drone].pose_id]
        cfg.droneNames[drone].position = poses[cfg.droneNames[drone].pose_id]
    # os.startfile(Flag_ue_executable_file)


    ALLOW_MAPF = True
    main_control = Control(cfg, 1, True)
    mapf = MAPF()
    mapf.l = l
    mapf.rho = rep
    mapf.add_deployment(deployment)
    main_control.mapf.append(mapf)
    allowed = set()
    coll = 0
    steps = 0
    f = open('experiment/result2.txt' , 'a', encoding = 'utf-8')
    new_poses = all_poses[1]
    trans, extra = Motill.simple(MultiDrones(main_control.cfg.all_drones).position, np.array(new_poses))
    for pair in trans:
        main_control.cfg.all_drones[pair[0]].target = new_poses[pair[1]]
        poses[pair[0]] = new_poses[pair[1]]
    for e in extra:
        main_control.cfg.all_drones[e].cur = collectors
        main_control.cfg.all_drones[e].position = collectors
        poses[e] = collectors
    mdrones = MultiDrones(main_control.cfg.all_drones)
    for i in tqdm(range(1000000)):
        if i > 6000:
            f.write(f'{l}, {rep}, infty, {coll}\n')
            break
        steps = i
        code = mapf.next_step(mdrones, i, False)
        if i % mapf.STEPS_PER_DISPATCH == 0:
            allowed = mapf.allowed
        if len(mapf.allowed) == 165:
            for i in range(len(mdrones.position)):
                dist = np.linalg.norm(mdrones.position[i] - mdrones.position, axis = 1)
                dist[i] = max(dist)
                dist[dist>1] = 0
                for e in extra:
                    if np.linalg.norm(mdrones.position[e], collectors) <= 2:
                        dist[e] = 0
                dist[dist>0] = 1
                # if dist.sum() > 0:
                #     print('coll!')
                coll += dist.sum()
        if code == 0:
            break
    f.write(f'{l}, {rep}, {steps}, {coll}\n')
    f.flush()
    f.close()
    print(f'{l}, {rep} has ended!')


if __name__ == "__main__":
    ls = list(np.arange(1,2,0.1))
    reps = list(np.arange(1,2,0.1))
    p = Pool(6)
    freeze_support()
    set_start_method('spawn', force=True)
    for l in ls:
        for rep in reps:
            p.apply_async(multi_step, (l,rep))
    p.close()
    p.join()
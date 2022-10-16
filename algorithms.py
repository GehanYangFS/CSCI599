# Implementation of MinDist and QuotaBalanced Algorithm

import math
from random import randint
from airsim import Vector3r
import collections
import numpy as np
import copy
from pointcloud import AlphabetPointCloud

def MinDist(poses: list[list[int]], dispatchers: list[list[int]]) -> dict[int, list[int]]:
    """Implementation of MinDist Algorithm in Shahram Ghandeharizadeh. 
        2022. Display of 3D Illuminations using Flying Light Specks

    Args:
        poses (list[list[int]]): positions of drone target points
        dispatchers (list[list[int]]): positions of drone dispathcer points

    Returns:
        dict[int, list[int]]: the deloyment of drones sorted by descending 
        distance for each dispatcher
    """    
    deploy = collections.defaultdict(list)
    for i in range(len(poses)):
        minDist = math.inf
        tgt = -1
        for j in range(len(dispatchers)):
            dist = Vector3r(*poses[i]).distance_to(Vector3r(*dispatchers[j]))
            if dist < minDist:
                minDist = dist
                tgt = j
        deploy[tgt].append(i)
    for dispather in deploy:
        deploy[dispather].sort(reverse=True, key = lambda v: Vector3r(*poses[v]).distance_to(Vector3r(*dispatchers[dispather])))
    return deploy


def QuotaBalanced(poses: list[list[int]], dispatchers: list[list[int]],  dispather_quota:list[int], f: int = 1, S: float = 4) -> dict[int, list[int]]:
    """Implementation of QuotaBalanced Algorithm in Shahram Ghandeharizadeh. 
        2022. Display of 3D Illuminations using Flying Light Specks

    Args:
        poses (list[list[int]]): positions of drone target points
        dispatchers (list[list[int]]): positions of drone dispathcer points
        dispather_quota (list[int]): drones available at each drone dispathcer
        f (int): 𝐹LS deloyment rate, FLSs per second
        S (float): FLS Speed

    Returns:
        dict[int, list[int]]: the deloyment of drones sorted by descending 
        distance for each dispatcher
    """    
    deploy = collections.defaultdict(list)
    ntarget = len(poses)
    ndispatcher = len(dispatchers)
    quota = [ntarget // (ndispatcher * f)] * ndispatcher
    active = [i for i in range(ndispatcher) if quota[i] and dispather_quota[i]]
    dispather_quota_copy = copy.deepcopy(dispather_quota)
    for i in range(len(poses)):
        idx = active[np.argmin([Vector3r(*poses[i]).distance_to(Vector3r(*dispatchers[j])) for j in active])]
        dist = Vector3r(*poses[i]).distance_to(Vector3r(*dispatchers[idx]))
        deploy[idx].append(i)
        dispather_quota_copy[idx] -= 1
        if dispather_quota_copy[idx] == 0:
            active.remove(int(idx))
        t = dist / S
        quota[idx] -= t
        if quota[idx] <= 0 and idx in active:
            active.remove(int(idx))
        if len(active) == 0:
            quota = [max(1, (ntarget - i - 1) // (ndispatcher * f))] * ndispatcher
            active = [i for i in range(ndispatcher) if quota[i] and dispather_quota[i]]
    for dispather in deploy:
        deploy[dispather].sort(reverse=True, key = lambda v: Vector3r(*poses[v]).distance_to(Vector3r(*dispatchers[dispather])))
    return deploy


if __name__ == '__main__':
    poses = []
    apc = AlphabetPointCloud(downwash=1)
    poses.extend(apc.query_point_cloud('U'))
    poses.extend(apc.query_point_cloud('S', [0, 25, 0]))
    poses.extend(apc.query_point_cloud('C', [0, 50, 0]))
    numDrones = len(poses)
    dispatchers = [
        [randint(0,100),randint(0,100),0] for _ in range(4)
    ]
    print(MinDist(poses, dispatchers))
    print(QuotaBalanced(poses, dispatchers, [numDrones // len(dispatchers) + 1 for _ in range(len(dispatchers))], 1, 4))
import json
from flags import Flag_ue_executable_settings_path
import collections
import numpy as np

class Drone:
    def __init__(self, name, pose_id, position, target) -> None:
        self.name = name
        self.pose_id = pose_id
        self.position = position
        self.target = target
        self.cur = position
        self.light = True
        self.rotor = True
        self.stand_by = False
        self.available = True

class Dispathcers:
    def __init__(self, deployment) -> None:
        self.dispathcers = deployment
        self.allowed = []
        self.allow_id = -1
        self.all = set([_v for _,v in deployment.items() for _v in v])
    
    @property
    def allow_dispathed(self):
        self.allow_id += 1
        for k,v in self.dispathcers.items():
            if self.allow_id >= len(v): # single dispatcher reaches limit
                continue
            self.allowed.append(v[::-1][self.allow_id])
        return set(self.allowed)
            


class MultiDrones:
    def __init__(self, drones: list[Drone]) -> None:
        self.position = np.zeros([len(drones), 3])
        self.target = np.zeros([len(drones), 3])
        for i in range(len(drones)):
            self.position[i] = drones[i].cur
            self.target[i] = drones[i].target

    def __len__(self):
        return len(self.position)

class Config:
    def __init__(self, baseSettingDir: str, outputDir: str) -> None:
        self.base = json.load(open(baseSettingDir))
        self.drone_number = 0
        self.outputDir = outputDir
        self.droneNames = {}
        self.dispatchers = collections.defaultdict(list)
        self.z_axis = -1

    @property
    def _drone_number(self) -> int:
        self.drone_number += 1
        return self.drone_number

    @property
    def all_drones(self) -> list[Drone]:
        return sorted(list(self.droneNames.values()), key=lambda i: i.pose_id)

    def _add_drone_with_position(self, x: float, y: float, z: float, pose_id: int, dispatcher: int) -> str:
        drone_name = 'Drone_{}'.format(self._drone_number)
        self.base['Vehicles'][drone_name] = {
            "VehicleType": "SimpleFlight",
            "X": x, "Y": y, "Z": self.z_axis
        }
        self.droneNames[drone_name] = Drone(drone_name, pose_id, np.array([x,y,self.z_axis]), None)
        self.dispatchers[dispatcher].append(drone_name)
        return drone_name

    def generateDrones(self, dispatchers: list[list[int]], quotaPerDispathcer: int, deployment: dict[int, list[int]] = {}) -> None:
        if len(deployment) == 0:
            pose_id = 0
            for _ in range(quotaPerDispathcer):
                for idx, dispatcher in enumerate(dispatchers):
                    drone_name = self._add_drone_with_position(*dispatcher, pose_id = pose_id, dispatcher = idx)
                    if idx == len(dispatchers) - 1:
                        self.droneNames[drone_name].stand_by = True
                    pose_id += 1
                    # self.z_axis -= 2
        else:
            for k,v in deployment.items():
                self.z_axis = -10
                for pose_id in v:
                    drone_name = self._add_drone_with_position(*dispatchers[k], pose_id = pose_id, dispatcher = k)
                    if k == max(list(deployment.keys())):
                        self.droneNames[drone_name].stand_by = True
                    pose_id += 1
                    # self.z_axis -= 2
        output = open(self.outputDir, 'w', encoding='utf-8')
        json.dump(self.base, output)


if __name__ == '__main__':
    config = Config('baseSettings.json', Flag_ue_executable_settings_path)
    config.generateDrones(
        [[0,0,0], [4,0,0], [4,4,0],[0,4,0]],
        100//4
    )
import json
from flags import Flag_ue_executable_settings_path
import collections

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
        

    def _add_drone_with_position(self, x: float, y: float, z: float, pose_id: int, dispatcher: int) -> None:
        drone_name = 'Drone_{}'.format(self._drone_number)
        self.base['Vehicles'][drone_name] = {
            "VehicleType": "SimpleFlight",
            "X": x, "Y": y, "Z": self.z_axis
        }
        self.droneNames[drone_name] = pose_id
        self.dispatchers[dispatcher].append(drone_name)

    def generateDrones(self, dispatchers: list[list[int]], quotaPerDispathcer: int, deployment: dict[int, list[int]] = {}) -> None:
        if len(deployment) == 0:
            pose_id = 0
            for _ in range(quotaPerDispathcer):
                for idx, dispatcher in enumerate(dispatchers):
                    self._add_drone_with_position(*dispatcher, pose_id = pose_id, dispatcher = idx)
                    pose_id += 1
                # self.z_axis -= 1
        else:
            for k,v in deployment.items():
                for pose_id in v:
                    self._add_drone_with_position(*dispatchers[k], pose_id = pose_id, dispatcher = k)
        output = open(self.outputDir, 'w', encoding='utf-8')
        json.dump(self.base, output)


if __name__ == '__main__':
    config = Config('baseSettings.json', Flag_ue_executable_settings_path)
    config.generateDrones(
        [[0,0,0], [4,0,0], [4,4,0],[0,4,0]],
        100//4
    )
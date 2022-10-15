from random import randint


class PointCloud:
    def __init__(self, src: str, numberOfPoints) -> None:
        self.src = src
        self.numberOfPoints = numberOfPoints

    def _read(self):
        points = []
        with open(self.src, 'r') as f:
            lines = f.readlines()
            for line in lines:
                points.append(line.strip().split(','))
        return points[:self.numberOfPoints]

    def _randPoints(self):
        output = []
        for i in range(self.numberOfPoints):
            output.append(
                [randint(90, 100), randint(90, 100), -randint(150, 200)])
        return output
    
    def getPoints(self):
        if self.src == "":
            return self._randPoints()
        else:
            return self._read()


if __name__ == '__main__':
    pc = PointCloud("", 100)
    print(pc.getPoints())

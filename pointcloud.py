from random import randint
from xmlrpc.client import boolean
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import matplotlib.pyplot as plt

class PointCloud:
    def __init__(self, src: str, numberOfPoints) -> None:
        self.src = src
        self.numberOfPoints = numberOfPoints

    def _read(self):
        points = []
        with open(self.src, 'r') as f:
            lines = f.readlines()
            for line in lines:
                points.append(line.strip().split(' '))
        return points[:self.numberOfPoints]

    def _randPoints(self):
        output = []
        for i in range(self.numberOfPoints):
            output.append(
                [randint(90, 100), randint(90, 100), -randint(150, 200)])
        return output
    
    def getPoints(self):
        if self.src == "":
            output =  self._randPoints()
        elif isinstance(self.src, list):
            return self.src
        else:
            output = [[float(i[0]),float(i[1]),float(i[2])] for i in self._read()]
        return output



class AlphabetPointCloud(): 
    def __init__(self, downwash: int = 3, size: int = 24, fontTypeSrc: str = "arial.ttf") -> None:
        self.size = size
        self.downwash =  downwash

        # Font
        self.font = ImageFont.truetype(fontTypeSrc, size = size, encoding='utf-8')

    def _query_image(self, alphabet: str):
        assert len(alphabet) == 1

        # Image
        image = Image.new("1", (self.size, self.size), "black")
        draw = ImageDraw.Draw(image)
        offset_w, offset_h = self.font.getoffset(alphabet)
        w, h = draw.textsize(alphabet, font=self.font)
        pos = ((self.size-w-offset_w)/2, (self.size-h-offset_h)/2)

        # Draw
        draw.text(pos, alphabet, "white", font=self.font)
        
        return image

    def query_matrix(self, alphabet: str):
        image = self._query_image(alphabet)
        return np.array(image, dtype=np.uint8)
    
    def query_point_cloud(self, alphabet: str, offset: list = [0,0,0], volume: int = 1, debug: boolean = False):
        mat = self.query_matrix(alphabet)
        pcs = []
        for i in range(len(mat)):
            for j in range(len(mat[0])):
                if (mat[i][j] == 1):
                    pcs.append([i + offset[0], j + offset[1], -100 + offset[2]])
                    for p in range(self.downwash * 2 + 1):
                        for q in range(-self.downwash, self.downwash + 1):
                            mat[i+p][j+q] = 0
        if debug:
            return pcs
        pcs = np.array(pcs, dtype=np.float32)
        diff = pcs[:, 0]
        pcs[:,2] += (diff - np.max(diff))
        pcs[:,0] = max(diff)
        out = pcs.copy()
        for _ in range(1, volume):
            pcs[:, 0] += 2 
            out = np.concatenate((out, pcs), axis = 0)
        return out

    def debug_point_cloud(self, alphabet: str):
        pcs = self.query_point_cloud(alphabet, debug=True)
        pcs_img = np.zeros([self.size,self.size])
        for pc in pcs:
            pcs_img[int(pc[0])][int(pc[1])] = 255
        plt.imshow(pcs_img, cmap='gray')
        plt.show()


if __name__ == '__main__':
    # pc = PointCloud("", 100)
    # print(pc.getPoints()) 

    apc = AlphabetPointCloud(downwash=1)
    apc.debug_point_cloud('B')
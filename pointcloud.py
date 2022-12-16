from random import randint
from xmlrpc.client import boolean
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import matplotlib.pyplot as plt
import os
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
        self.fontTypeSrc = fontTypeSrc
        # Font
        fontpath = os.path.join(r'C:\Windows\Fonts',fontTypeSrc)
        self.font = ImageFont.truetype(fontpath, size = size, encoding='utf-8')

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

    def query_matrix(self, alphabet: str, savefig: bool = False):
        image = self._query_image(alphabet)
        if savefig:
            fig = plt.imshow(image, cmap='gray')
            fig.axes.get_xaxis().set_visible(False)
            fig.axes.get_yaxis().set_visible(False)
            plt.savefig(f"{alphabet}_{self.fontTypeSrc}.png", transparent=True, dpi = 300)
        return np.array(image, dtype=np.uint8)
    
    def query_point_cloud(self, alphabet: str, offset: list = [0,0,0], volume: int = 1, debug: boolean = False):
        mat = self.query_matrix(alphabet)
        pcs = []
        for i in range(len(mat)):
            for j in range(len(mat[0])):
                if (mat[i][j] == 1):
                    pcs.append([i + offset[0], j + offset[1], -50 + offset[2]])
                    for p in range(self.downwash * 2 + 1):
                        for q in range(-self.downwash, self.downwash + 1):
                            if i+p < len(mat) and j + q < len(mat[0]):
                                mat[i+p][j+q] = 0
        if debug:
            return pcs
        pcs = np.array(pcs, dtype=np.float32)
        diff = pcs[:, 0]
        pcs[:,2] += (diff - np.max(diff))
        pcs[:,0] = max(diff)
        out = pcs.copy()
        for _ in range(1, volume):
            pcs[:, 0] += 5
            out = np.concatenate((out, pcs), axis = 0)
        return out

    def debug_point_cloud(self, alphabet: str, savefig: bool = False):
        pcs = self.query_point_cloud(alphabet, debug=True)
        pcs_img = np.zeros([self.size,self.size])
        for pc in pcs:
            pcs_img[int(pc[0])][int(pc[1])] = 255
        if savefig:
            fig = plt.imshow(pcs_img, cmap='gray')
            fig.axes.get_xaxis().set_visible(False)
            fig.axes.get_yaxis().set_visible(False)
            plt.savefig("experiment/U_1.pdf")
        return pcs_img


if __name__ == '__main__':
    # pc = PointCloud("", 100)
    # print(pc.getPoints()) 

    all_poses = []
    apc = AlphabetPointCloud(downwash=2, size = 48)
    apc.debug_point_cloud('U')
    # frames = ['USC', 'TROJANS']
    # volumes = {
    #     'USC': 5,
    #     'TROJANS': 2
    # }
    # offsets = {
    #     'USC': 20,
    #     'TROJANS': 20
    # }
    # for frame in frames:
    #     pose = []
    #     for idx, c in enumerate(frame):
    #         pose.extend(apc.query_point_cloud(alphabet=c, offset = [0, offsets[frame] * idx, 0], volume=volumes[frame]))
    #     all_poses.append(pose)
    #     with open(f'pointclouds/{frame}.pc','w',encoding='utf-8') as f:
    #         for p in pose:
    #             f.write(f'{int(p[0])},{int(p[1])},{int(p[2])}\n')
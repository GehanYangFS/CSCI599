
import sys
import os
import cv2 as cv
# getting the name of the directory
# where the this file is present.
current = os.path.dirname(os.path.realpath(__file__))

# Getting the parent directory name
# where the current directory is present.
parent = os.path.dirname(os.path.dirname(current))

# adding the parent directory to
# the sys.path.
sys.path.append(parent)

from pointcloud import AlphabetPointCloud
import numpy as np
import matplotlib.pyplot as plt


all_fonts = [
    'arial.ttf',
    'calibri.ttf',
    'georgia.ttf',
    'BROADW.TTF',
    'ITCEDSCR.TTF',
    'BRADHITC.TTF',
    'COOPBL.TTF',
    'OLDENGL.TTF'
]

# for font in all_fonts:
#     apc = AlphabetPointCloud(downwash=2, size = 480, fontTypeSrc=font)
#     apc.query_matrix('A', True)

mats = []
for font in all_fonts:
    apc = AlphabetPointCloud(downwash=2, size = 480, fontTypeSrc=font)
    mats.append(apc.query_matrix('A'))
mat = np.concatenate(mats, axis = 1)
fig = plt.imshow(mat, cmap='gray')
fig.axes.get_xaxis().set_visible(False)
fig.axes.get_yaxis().set_visible(False)
plt.savefig(f"A.png", transparent=True, dpi = 300)
mat[mat>0]=255
cv.imwrite(f"A.png", mat)

word = 'USC'
apc = AlphabetPointCloud(downwash=2, size = 480, fontTypeSrc='arial.ttf')
mats = []
for a in word:
    mats.append(apc.query_matrix(a))
mat = np.concatenate(mats, axis = 1)
fig = plt.imshow(mat, cmap='gray')
fig.axes.get_xaxis().set_visible(False)
fig.axes.get_yaxis().set_visible(False)
plt.savefig(f"{word}.png", transparent=True,dpi = 300)
prev_mat = mat

word = 'USC'
apc = AlphabetPointCloud(downwash=2, size = 480, fontTypeSrc='arial.ttf')
mats = []
for a in word:
    mats.append(apc.debug_point_cloud(a))
mats.append(mats[-1])
mats[-1] = np.zeros_like(mats[-1])
mat = np.concatenate(mats, axis = 1)
fig = plt.imshow(mat, cmap='gray')
fig.axes.get_xaxis().set_visible(False)
fig.axes.get_yaxis().set_visible(False)
plt.savefig(f"{word}_debug.png", transparent=True,dpi = 300)

part = mat[300:400,1250:1350]
mask = cv.resize(part, (300, 300), fx=0, fy=0, interpolation=cv.INTER_LINEAR)
if mat is None is None:
    print('Failed to read picture')
    sys.exit()
    
#放大后局部图的位置img[210:410,670:870]
mat[90:390,1550:1850]=mask

#画框并连线
cv.rectangle(mat,(1250, 300),(1350, 400),(255,255,255),1)
cv.rectangle(mat,(1550,90),(1850,390),(255,255,255),1)
img = cv.line(mat,(1350,400),(1550,390),(255,255,255))
img = cv.line(mat,(1350,300),(1550,90),(255,255,255))
#展示结果
# cv.imshow('img',img)
# cv.waitKey(0)
# cv.destroyAllWindows()

img = np.concatenate([prev_mat, img], axis=1)
img[img>0]=255
cv.imwrite(f"{word}_debug.png", img)


   
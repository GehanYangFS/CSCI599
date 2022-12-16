import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from collections import defaultdict

f = open(r'C:\Users\shenb\Downloads\CSCI-599\CSCI599\experiment\result2.txt', 'r', encoding='utf-8')
lines = f.readlines()
f.close()
data1 = []
data2 = []
data3 = defaultdict(list)
for line in lines:
    l, rep, steps, coll = line.split(',')
    l = float(l)
    rep = float(rep)
    if 'infty' in steps:
        steps = np.inf
    else:
        steps = float(steps)
    coll = float(coll)
    data1.append([l,rep,steps])
    data2.append([l,rep,coll])
    data3[l].append([rep, steps * l])

data1 = np.array(data1)
data2 = np.array(data2)

fig, ax = plt.subplots()
# ax.set_yscale('log')
for k,v in data3.items():
    v = sorted(v)
    v = np.array(v)
    line, = ax.plot(v[:,0], v[:,1])
ax.legend([f'l={i:.1f}' for i in data3.keys()])
ax.set_ylabel('Total Distance')
ax.set_xlabel('Repulsive Radius')
plt.show()


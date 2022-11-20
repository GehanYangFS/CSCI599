import numpy as np



class Motill:
    @classmethod
    def simple(cls, pc1: np.ndarray, pc2: np.ndarray):
        if len(pc1) < len(pc2):
            raise Exception("number of drones in frame 1 should be large or equal to number of drones in frame 2")
        all_pairs = []
        for i in range(len(pc1)):
            all_dist = np.linalg.norm(pc1[i] - pc2, axis = 1)
            for j in range(len(pc2)):
                all_pairs.append([i,j,all_dist[j]])
        all_pairs = sorted(all_pairs, key=lambda i: i[2])
        seen1 = set()
        seen2 = set()
        ret = []
        for p in all_pairs:
            if p[0] in seen1 or p[1] in seen2:
                continue
            ret.append(p[:2])
            seen1.add(p[0])
            seen2.add(p[1])
        return ret, set([i for i in range(len(pc1))]) - seen1



    
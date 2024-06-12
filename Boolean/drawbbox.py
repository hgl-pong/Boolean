from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from itertools import product, combinations

import time
import math

fig = plt.figure()
ax = fig.add_subplot(projection = '3d')
ax.set_aspect("equal")


start = time.time()
# draw cube

r = [-10, 1]

maxa=[0.016433,0.5,0.187976375]
mina=[-0.5,-0.5,-0.5]

maxb=[0.4999,-0.184731007,0.130559489]
minb=[0.1155441494,-0.49,-0.271037519]

ax.bar3d(mina[0], mina[1], mina[2], maxa[0]-mina[0], maxa[1]-mina[1], maxa[2]-mina[2],color="green",zsort='average',edgecolor='white',linewidth=0.5,alpha=0.5) 
ax.bar3d(minb[0], minb[1], minb[2], maxb[0]-minb[0], maxb[1]-minb[1], maxb[2]-minb[2],color="b",zsort='average',edgecolor='white',linewidth=0.5,alpha=0.5) 


print('spend_time:', time.time() - start)

# draw sphere
#u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
#x = np.cos(u)*np.sin(v)
#y = np.sin(u)*np.sin(v)
#z = np.cos(v)
#ax.plot_wireframe(x, y, z, color="r")

# draw a point
#ax.scatter([0], [0], [0], color="g", s=100)

# draw a vector
#from matplotlib.patches import FancyArrowPatch
#from mpl_toolkits.mplot3d import proj3d


# class Arrow3D(FancyArrowPatch):

    # def __init__(self, xs, ys, zs, *args, **kwargs):
        # FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        # self._verts3d = xs, ys, zs

    # def draw(self, renderer):
        # xs3d, ys3d, zs3d = self._verts3d
        # xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        # self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        # FancyArrowPatch.draw(self, renderer)

# a = Arrow3D([0, 1], [0, 1], [0, 1], mutation_scale=20,
           # lw=1, arrowstyle="-|>", color="k")
# ax.add_artist(a)

plt.show()

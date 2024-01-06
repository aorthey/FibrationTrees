import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc

kDPI = 30
kNumSamples = 50
R=1.2
r=0.5

def DrawVerticalCircleOnTorus(ax, angle_phi):
  phi = np.linspace(angle_phi, angle_phi, endpoint=True, num=kNumSamples)
  theta = np.linspace(0, 2 * np.pi, endpoint=True, num=kNumSamples)
  phi, theta = np.meshgrid(phi, theta)

  x = np.cos(theta) * (R + r * np.cos(phi))
  y = np.sin(theta) * (R + r * np.cos(phi))
  z = r * np.sin(phi)

  ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(z)))
  ax.plot_surface(x, y, z, color='g', edgecolors='g', antialiased=True, alpha=0, rstride=2, cstride=2)

def DrawTorus(ax):
  phi = np.linspace(0, 2 * np.pi, endpoint=True, num=kNumSamples)
  theta = np.linspace(0, 2 * np.pi, endpoint=True, num=kNumSamples)
  phi, theta = np.meshgrid(phi, theta)

  x = np.cos(theta) * (R + r * np.cos(phi))
  y = np.sin(theta) * (R + r * np.cos(phi))
  z = r * np.sin(phi)

  ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(z)))
  ax.plot_surface(x, y, z, color='k', edgecolors='k', antialiased=True, alpha=0, rstride=2, cstride=2)


fig = plt.figure(dpi=kDPI)

plt.rc('text', usetex=True)
plt.rc('font', family='serif')

ax = fig.add_subplot(1, 1, 1, projection='3d')

DrawTorus(ax)
DrawVerticalCircleOnTorus(ax, 0.5)

fig.tight_layout()
ax.axis('tight')
ax.axis('off')
plt.show()


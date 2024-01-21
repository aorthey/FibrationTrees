import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc, rcParams
from Paper_plot_fibration_types import *

fig = plt.figure(figsize=(30,10))
ax1 = fig.add_subplot(1, 1, 1, projection='3d', computed_zorder=False)

#Debug
# fig.patch.set_facecolor('xkcd:mint green')
# ax1.set_facecolor('xkcd:salmon')

plt.rc('text', usetex=True)
plt.rc('font', family='serif')

torus = Torus(ax1, major_radius=1.2, minor_radius=0.5)
torus.Draw('grey')

robot = Robot(ax1)

def DrawRobotLineOnTorus(x1, x2, color, endpoints=False, markersize=100, linewidth=8):
  x = np.linspace(x1, x2, 100)
  phi1 = []
  theta1 = []
  phi2 = []
  theta2 = []
  for k in range(0, len(x)):
    [[t1a, t2a], [t1b, t2b]] = robot.IK(x[k], 1)
    theta1.append(t1a)
    phi1.append(t2a)
    theta2.append(t1b)
    phi2.append(t2b)

  torus.DrawLine(theta1, phi1, color=color, linewidth=linewidth)
  torus.DrawLine(theta2, phi2, color=color, linewidth=linewidth)
  torus.DrawPoint(theta2[0], phi2[0], color=color, markersize=markersize)
  torus.DrawPoint(theta1[0], phi1[0], color=color, markersize=markersize)
  if endpoints:
    torus.DrawPoint(theta2[-1], phi2[-1], color=color, markersize=markersize)
    torus.DrawPoint(theta1[-1], phi1[-1], color=color, markersize=markersize)

torus.zorder=2
DrawRobotLineOnTorus(-1.5, np.sqrt(3), color='violet', linewidth=10,
    markersize=200)
torus.zorder=3
DrawRobotLineOnTorus(-0.0, 1.4, color='purple', linewidth=15, markersize=500,
    endpoints=True)

ax1.set_xlim(np.array([-2,2])*0.6)
ax1.set_ylim(np.array([-2,2])*0.6)
ax1.set_zlim(np.array([-2,2])*0.28)
ax1.axis('off')
#
ax1.view_init(elev=30, azim=-90)

plt.tight_layout()
fig.subplots_adjust(left=0, bottom=0.0, right=1, top=1, wspace=0, hspace=0)

plt.savefig("FibrationTypes3_Torus.svg", bbox_inches='tight', dpi=kDPI)
plt.savefig("FibrationTypes3_Torus.png", bbox_inches='tight', dpi=kDPI)
plt.show()

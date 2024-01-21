import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc
from Paper_plot_fibration_types import *

fig = plt.figure(dpi=kDPI)
ax2 = fig.add_subplot(1, 1, 1)

plt.rc('text', usetex=True)
plt.rc('font', family='serif')
################################################################################
## plot 2d robot
################################################################################
robot = Robot(ax2)
alpha=0.2
for d in [-0.0, +0.3, +0.6, 0.9, 1.2, 1.5]:
  robot.zorder=robot.zorder+2
  robot.Draw(d, 1, alpha=alpha)
  alpha+=0.1

## Plot path segment
ax2.plot([0.0, 1.5], [1, 1], color='purple', linewidth=12, zorder=2)
ax2.scatter(0.0, 1, color='purple', s=250, zorder=2)
ax2.scatter(1.5, 1, color='purple', s=250, zorder=2)

ax2.plot([-1.0, 3.0], [1, 1], color='violet', linewidth=10, zorder=0)
ax2.plot([+np.sqrt(3), 3.0], [1, 1], color='red', linewidth=10, zorder=1)
ax2.set_xlim(np.array([-1.25,3.25]))
ax2.set_ylim(np.array([-0.25,1.25]))
ax2.set_aspect('equal')

plt.subplots_adjust(left=0.0, right=1, top=1, bottom=0.0)
plt.savefig("FibrationTypes3_Robot.svg", bbox_inches='tight', dpi=kDPI)
plt.savefig("FibrationTypes3_Robot.png", bbox_inches='tight', dpi=kDPI)
plt.show()


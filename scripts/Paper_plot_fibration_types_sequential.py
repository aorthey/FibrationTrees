import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc
from Paper_plot_fibration_types import *

fig = plt.figure(dpi=kDPI)
ax = fig.add_subplot(projection='3d', computed_zorder=False)

plt.rc('text', usetex=True)
plt.rc('font', family='serif')

phiPosition = 0.0
thetaRestrictionStart = -2.5
thetaRestrictionEnd = -1.4
zPositionCircle=-1.0
zPositionTorus=+1.0
kPathLineWidth=3

pathColor = 'g'
fibrationColor = 'g'
fibrationColor = 'palegreen'

torus = Torus(ax, z=zPositionTorus, major_radius=1.2, minor_radius=0.5)
torus.Draw('grey')
torus.DrawPhiPatch(thetaRestrictionStart, thetaRestrictionEnd,
    color=fibrationColor, circleColor='grey', linewidth=0.5)
torus.DrawThetaCircle(phiPosition, color=fibrationColor)
torus.zorder=2
torus.DrawThetaPath(phiPosition, thetaRestrictionStart, thetaRestrictionEnd,
    color=pathColor, linewidth=kPathLineWidth)

## Draw Circle and Path on Circle
circle = Circle(ax, z=zPositionCircle, radius=1.5)
circle.Draw(fibrationColor)
circle.zorder=2
circle.DrawSegment(thetaRestrictionStart, thetaRestrictionEnd, pathColor)

ax.set_aspect('auto')
fig.tight_layout()
ax.set_xlim(np.array([-2,2])*0.65)
ax.set_ylim(np.array([-2,2])*0.65)
ax.set_zlim(np.array([-2,2])*0.45)
plt.subplots_adjust(left=0.0, right=1, top=1, bottom=0.0)
ax.axis('off')

ax.view_init(elev=25, azim=-90)

plt.savefig("FibrationTypes1.svg", bbox_inches='tight', dpi=kDPI)
plt.savefig("FibrationTypes1.png", bbox_inches='tight', dpi=kDPI)

plt.show()


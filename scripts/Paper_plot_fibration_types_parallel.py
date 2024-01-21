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

phiPosition = -0.0 ## for embedding
thetaPosition = -0.8 ## for embedding
thetaRestrictionStart = -2.5
thetaRestrictionEnd = -1.4
phiRestrictionStart = +0.2
phiRestrictionEnd = +1.5
zPositionCircle=-0.5
zPositionTorus=+0.5
kPathLineWidth=3

thetaPathColor = 'g'
thetaComponentColor = 'palegreen'

phiPathColor = 'orange'
phiComponentColor = 'moccasin'

intersectionColor='limegreen'

torus = Torus(ax, z=zPositionTorus, major_radius=1.2, minor_radius=0.5)
torus.Draw('grey')
torus.DrawThetaCircle(phiPosition, color=thetaComponentColor)
torus.zorder=2
torus.DrawPhiCircle(thetaPosition, color=phiComponentColor)

## Draw Restrictions
torus.zorder=3
torus.DrawPhiPatch(thetaRestrictionStart, thetaRestrictionEnd,
    color=thetaComponentColor, circleColor='grey', linewidth=0.5 )
torus.DrawThetaPatch(phiRestrictionStart, phiRestrictionEnd,
    color=phiComponentColor, circleColor='grey', linewidth=0.5)

## Draw Intersection
torus.zorder=4
torus.DrawPatch( thetaRestrictionStart, thetaRestrictionEnd,
phiRestrictionStart, phiRestrictionEnd, color=intersectionColor)

## Draw Embedded Path on Restrictions
torus.zorder=5
torus.DrawPhiPath(thetaPosition, phiRestrictionStart, phiRestrictionEnd,
    color=phiPathColor, linewidth=kPathLineWidth)
torus.DrawThetaPath(phiPosition, thetaRestrictionStart, thetaRestrictionEnd,
    color=thetaPathColor, linewidth=kPathLineWidth)


## Draw Circle and Path on Circle
circleRadius = 0.85
circle1 = Circle(ax, x=+0.9, y=0.0, z=-1.0, radius=circleRadius)
circle1.Draw(phiComponentColor)
circle1.DrawSegment(phiRestrictionStart, phiRestrictionEnd, phiPathColor)

circle2 = Circle(ax, x=-0.9, y=0.0, z=-1.0, radius=circleRadius)
circle2.Draw(thetaComponentColor)
circle2.DrawSegment(thetaRestrictionStart, thetaRestrictionEnd, thetaPathColor)

ax.set_aspect('auto')
fig.tight_layout()
ax.set_xlim(np.array([-2,2])*0.8)
ax.set_ylim(np.array([-2,2])*0.8)
ax.set_zlim(np.array([-2,2])*0.18)
plt.subplots_adjust(left=0.0, right=1, top=1, bottom=0.0)
ax.axis('off')

ax.view_init(elev=30, azim=-90)

plt.savefig("FibrationTypes2.svg", bbox_inches='tight', dpi=kDPI)
plt.savefig("FibrationTypes2.png", bbox_inches='tight', dpi=kDPI)

plt.show()


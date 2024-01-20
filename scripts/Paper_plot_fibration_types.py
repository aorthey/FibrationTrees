import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc

kDPI = 150
kNumSamples = 50
# R=1.2
# r=0.5

kLineWidth = 3
kMarkerSize = 60

#class Arrow:
#https://stackoverflow.com/questions/63546097/3d-curved-arrow-in-python

class Torus:
  #        | Phi
  #Theta   |
  #----->  v
  def __init__(self, ax, x=0.0, y=0.0, z=0.0, major_radius=1.0, minor_radius=0.5):
    self.ax = ax
    self.x_position = x
    self.y_position = y
    self.z_position = z
    self.major_radius = major_radius
    self.minor_radius = minor_radius
    self.zorder=1

  def Draw(self, color='k'):
    phi = np.linspace(0, 2 * np.pi, endpoint=True, num=kNumSamples)
    theta = np.linspace(0, 2 * np.pi, endpoint=True, num=kNumSamples)
    phi, theta = np.meshgrid(phi, theta)
    [x, y, z] = self.GenerateSamples(phi, theta)
    self.ax.plot_surface(x, y, z, color=color, edgecolors=color, alpha=0, rstride=2, cstride=2, linewidth=0.3)
    self.ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(z)))

  def GenerateSamples(self, phi, theta):
    R = self.major_radius
    r = self.minor_radius
    x = np.cos(theta) * (R + r * np.cos(phi)) + self.x_position
    y = np.sin(theta) * (R + r * np.cos(phi)) + self.y_position
    z = r * np.sin(phi) + self.z_position
    return [x, y, z]

  def DrawPatch(self, thetaStart, thetaEnd, phiStart, phiEnd, color='k',
      alpha=0.5):
    theta = np.linspace(thetaStart, thetaEnd, endpoint=True, num=kNumSamples)
    phi = np.linspace(phiStart, phiEnd, endpoint=True, num=kNumSamples)
    phi, theta = np.meshgrid(phi, theta)
    [x, y, z] = self.GenerateSamples(phi, theta)
    for i in range(2):
      self.ax.plot_surface(x, y, z, color=color, rstride=3, cstride=3, alpha=alpha,
          linewidth=0, edgecolor=color, zorder=self.zorder, shade=False,
          antialiased=True)
    self.ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(z)))

  def DrawPoint(self, theta, phi, color='g', markersize=kMarkerSize):
    [x, y, z] = self.GenerateSamples(phi, theta)
    self.ax.scatter3D(x, y, z, s=markersize, color=color, zorder=self.zorder)

  def DrawLine(self, theta, phi, color='m', linewidth=kLineWidth):
    #phi = np.linspace(phiStart, phiEnd, endpoint=True, num=kNumSamples)
    [x, y, z] = self.GenerateSamples(phi, theta)
    self.ax.plot3D(x, y, z, linewidth=linewidth, color=color, zorder=self.zorder)
  ################################################################################
  ## Draw vertical line along Phi dimension (around minor axis)
  ################################################################################
  def DrawPhiLine(self, theta, phiStart, phiEnd, color='m',
      linewidth=kLineWidth):
    phi = np.linspace(phiStart, phiEnd, endpoint=True, num=kNumSamples)
    [x, y, z] = self.GenerateSamples(phi, theta)
    self.ax.plot3D(x, y, z, linewidth=linewidth, color=color, zorder=self.zorder)

  def DrawPhiCircle(self, theta, linewidth=kLineWidth, color='m'):
    self.DrawPhiLine(theta, 0, 2*np.pi, linewidth=linewidth, color=color)

  def DrawPhiPatch(self, thetaStart, thetaEnd, color='m', circleColor='k',
      linewidth=kLineWidth):
    self.DrawPatch(thetaStart, thetaEnd, 0, 2*np.pi, color=color)
    self.DrawPhiCircle(thetaStart, color=circleColor, linewidth=linewidth)
    self.DrawPhiCircle(thetaEnd, color=circleColor, linewidth=linewidth)

  def DrawPhiPath(self, theta, phiStart, phiEnd, color='m', linewidth=kLineWidth):
    self.DrawPhiLine(theta, phiStart, phiEnd, linewidth=linewidth, color=color)
    self.DrawPoint(theta, phiStart, color=color)
    self.DrawPoint(theta, phiEnd, color=color)

  ################################################################################
  ## Draw horizontal line along Theta dimension (around major axis)
  ################################################################################
  def DrawThetaLine(self, phi, thetaStart, thetaEnd, color='m',
      linewidth=kLineWidth):
    theta = np.linspace(thetaStart, thetaEnd, endpoint=True, num=kNumSamples)
    [x, y, z] = self.GenerateSamples(phi, theta)
    self.ax.plot3D(x, y, z, linewidth=linewidth, color=color, zorder=self.zorder)

  def DrawThetaCircle(self, phi, linewidth=kLineWidth, color='m'):
    self.DrawThetaLine(phi, 0, 2*np.pi, linewidth=linewidth, color=color)

  def DrawThetaPatch(self, phiStart, phiEnd, color='m', circleColor='k',
      linewidth=kLineWidth):
    self.DrawPatch(0, 2*np.pi, phiStart, phiEnd, color=color)
    self.DrawThetaCircle(phiStart, color=circleColor, linewidth=linewidth)
    self.DrawThetaCircle(phiEnd, color=circleColor, linewidth=linewidth)

  def DrawThetaPath(self, phi, thetaStart, thetaEnd, color='m', linewidth=kLineWidth):
    self.DrawThetaLine(phi, thetaStart, thetaEnd, linewidth=linewidth, color=color)
    self.DrawPoint(thetaStart, phi, color=color)
    self.DrawPoint(thetaEnd, phi, color=color)

################################################################################
## Circle Drawing
################################################################################
class Circle:
  markersize = kMarkerSize
  def __init__(self, ax, x=0.0, y=0.0, z=0.0, radius=1.0):
    self.ax = ax
    self.x_position = x
    self.y_position = y
    self.z_position = z
    self.radius = radius
    self.zorder = 1

  def DrawSegment(self, start, goal, color):
    self.__DrawCircleSegment(start, goal, color=color)
    self.__DrawCirclePoint(start, color=color, markersize=self.markersize)
    self.__DrawCirclePoint(goal, color=color, markersize=self.markersize)
  
  def Draw(self, color):
    self.__DrawCircleSegment(0, 2*np.pi, color=color)

  def __DrawCircleSegment(self, thetaStart, thetaEnd, color):
    theta = np.linspace(thetaStart, thetaEnd, endpoint=True, num=kNumSamples)
    x = self.radius * np.cos(theta) + self.x_position
    y = self.radius * np.sin(theta) + self.y_position
    self.ax.plot3D(x, y, self.z_position, linewidth=kLineWidth, color=color,
        zorder=self.zorder)

  def __DrawCirclePoint(self, theta, color='g', markersize=3):
    x = self.radius * np.cos(theta) + self.x_position
    y = self.radius * np.sin(theta) + self.y_position
    self.ax.scatter3D(x, y, self.z_position, s=markersize, color=color,
        zorder=self.zorder)

kRobotColor = 'grey'
kRobotLineWidth = 10
kRobotMarkerSize = 20
kRobotInnerMarkerSize = 10*kRobotMarkerSize
class Robot:
  def __init__(self, ax):
    self.l1 = 1
    self.l2 = 1
    self.ax = ax
    self.zorder=1

  def FK(self, theta1, theta2):
    x = self.l1 * np.cos(theta1) + self.l2 * np.cos(theta1 + theta2)
    y = self.l1 * np.sin(theta1) + self.l2 * np.sin(theta1 + theta2)
    return [x, y]

  def IK(self, x, y):
    cosTheta2 = (x**2 + y**2 - self.l1**2 - self.l2**2) / 2*self.l1*self.l2
    if cosTheta2 > 1:
      return None
    theta2_a = +np.arccos(cosTheta2)
    k1_a = self.l1 + self.l2 * np.cos(theta2_a)
    k2_a = self.l2 * np.sin(theta2_a)
    theta1_a = np.arctan2(y, x) - np.arctan2(k2_a, k1_a)

    theta2_b = -np.arccos(cosTheta2)
    k1_b = self.l1 + self.l2 * np.cos(theta2_b)
    k2_b = self.l2 * np.sin(theta2_b)
    theta1_b = np.arctan2(y, x) - np.arctan2(k2_b, k1_b)
    return [[theta1_a, theta2_a], [theta1_b, theta2_b]]

  def DrawFromConfig(self, theta1, theta2, color=kRobotColor, alpha=1):
    x0 = 0
    y0 = 0

    x1 = self.l1 * np.cos(theta1)
    y1 = self.l1 * np.sin(theta1)

    x2 = x1 + self.l2 * np.cos(theta1 + theta2)
    y2 = y1 + self.l2 * np.sin(theta1 + theta2)
    self.ax.plot([x0, x1], [y0, y1], marker = 'o', color=color,
        linewidth=kRobotLineWidth, markersize=kRobotMarkerSize, alpha=alpha,
        zorder=self.zorder)
    self.ax.plot([x1, x2], [y1, y2], marker = 'o', color=color,
        linewidth=kRobotLineWidth, markersize=kRobotMarkerSize, alpha=alpha,
        zorder=self.zorder)

    self.ax.scatter(x0, y0, marker = 'o', color='k', s=3*kRobotInnerMarkerSize,
        zorder=self.zorder-1)
    self.ax.scatter(x0, y0, marker = 'o', color='w', s=kRobotInnerMarkerSize,
        zorder=self.zorder+2)
    self.ax.scatter(x1, y1, marker = 'o', color='w', s=kRobotInnerMarkerSize,
        zorder=self.zorder+2)
    self.ax.scatter(x2, y2, marker = 'o', color='w', s=kRobotInnerMarkerSize,
        zorder=self.zorder+2)

  def Draw(self, x, y, color=kRobotColor, alpha=1):
    [[theta1_a, theta2_a], [theta1_b, theta2_b]] = self.IK(x, y)
    self.DrawFromConfig(theta1_a, theta2_a, color=color, alpha=alpha)


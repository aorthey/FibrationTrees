import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from circle_fitting_3d import Circle3D
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc
import matplotlib.colors as mcolors
import matplotlib.tri as mtri
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


plt.rc('text', usetex=True)
plt.rc('font', family='serif')


### TODO
### [x] Ensure that only the fibers from the spherical solution path are shown
### [x] Ensure that no disconnected edges occur.
### [ ] Put goal position on S3 to the correct end position
### [ ] Understand why different interpolations happen
### [ ] Better antialiasing of paths

import yaml
import argparse
import random

kColorMapFiber = plt.get_cmap('viridis')

## Problem1: Why does interpolation sometimes stray away from edge fiber?
##          - Interpolation might be different and produce other/shorter path?
## Problem2: Why is search tree on SO3 sometimes disconnected?

def get_sequential_color(index, max_indices):
  return kColorMapFiber(index/(max_indices-1))

# def get_random_color(index):
#     clipped_index = max(min(index, len(color_tablet)-1), 0)
#     return color_tablet[clipped_index]

kDPI = 150
kNumSamples = 50
kLineWidth = 2
kLineWidthSpherePath = 6
kLineWidthHopfFiber = 0.5
kMarkerSize = 5
kMarkerSizeHopfStartPoint = 20
kMarkerSizeHopfGoalPoint = 20

kMarkerSizeSpherePoint = 100

kColorSpherePoint = [0,1,0]
kColorSphereStartPoint = [0,1,0]
kColorSphereGoalPoint = [1,0,0]
kColorHopfEdge = [0,0,0,0.5]

kColorSphereLines = [0.8,0.8,0.8,0.7]
kColorSphere = [1,1,1,0.05]

def DrawEdgeFromPoints(ax, points, marker_size, line_width, color, zorder):
  N = points.shape[0]
  t = np.arange(N)  # simple assumption that data was sampled in regular steps
  x = points[:,0]
  y = points[:,1]
  z = points[:,2]
  print(x,y,z)
  #ax.scatter3D(x[0], y[0], z[0], s=marker_size, color=color, zorder=zorder)
  fitx = np.polyfit(t, x, 3)
  fity = np.polyfit(t, y, 3)
  fitz = np.polyfit(t, z, 3)
  xx = np.polyval(fitx, t)
  yy = np.polyval(fity, t)
  zz = np.polyval(fitz, t)
  #ax.plot3D(xx, yy, zz, lw=line_width, color=color, zorder=zorder)
  #cmap=cm.ScalarMappable(norm=zz, cmap=plt.get_cmap('viridis'))

  
  # colors=[]
  # for i in np.arange(points.shape[0]):
  #   colors.append('#101')
  
  for i in np.arange(N-1):
    ax.plot3D(xx[i:i+2], yy[i:i+2], zz[i:i+2], lw=line_width, color=get_sequential_color(i, N))

  #ax.plot3D(xx, yy, zz, lw=line_width, color=colors, zorder=zorder)

def CartesianToSpherical(point):
  x = point[0]
  y = point[1]
  z = point[2]
  r = np.sqrt(x*x + y*y + z*z);
  theta = np.arctan2(y,x);
  phi = np.arccos(z/r);
  return [theta, phi]

def SphericalToCartesian(radius, theta, phi):
  if np.abs(phi-np.pi).any() < 1e-5:
    phi = phi - np.pi/8
  x = radius * np.sin(phi) * np.cos(theta);
  y = radius * np.sin(phi) * np.sin(theta);
  z = radius * np.cos(phi);
  return [x,y,z]

class Sphere:
  #        | Phi
  #Theta   |
  #----->  v
  def __init__(self, ax, x=0.0, y=0.0, z=-0.0, radius=1.0):
    self.ax = ax
    self.x_position = x
    self.y_position = y
    self.z_position = z
    self.radius = radius
    self.zorder=1

  def Draw(self):
    theta = np.linspace(-np.pi, np.pi, endpoint=True, num=kNumSamples)
    phi = np.linspace(0, np.pi, endpoint=True, num=kNumSamples)
    theta, phi = np.meshgrid(theta, phi)
    [x, y, z] = self.GenerateSamples(theta, phi)
    self.ax.plot_surface(x, y, z, color=kColorSphere, edgecolors=kColorSphereLines, rstride=2, cstride=2, linewidth=0.3)
    self.ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(z)))

  def DrawPoint(self, point, zorder = 1):
    self.ax.scatter3D(point[0], point[1], point[2], s=kMarkerSize, color=kColorSpherePoint, zorder=zorder)

  def DrawSpecialPoint(self, point, color, markersize=10, zorder=1):
    self.ax.scatter3D(point[0], point[1], point[2], s=markersize, color=color, zorder=zorder)

  def DrawLine(self, a, b, color, zorder=1):
    self.ax.plot3D([a[0],b[0]], [a[1],b[1]], [a[2],b[2]], lw=kLineWidthSpherePath, color=color, zorder=zorder)

  def DrawEdge(self, points, color, zorder=1):
    N = points.shape[0]
    t = np.arange(N)  # simple assumption that data was sampled in regular steps
    x = points[:,0]
    y = points[:,1]
    z = points[:,2]
    print(x,y,z)
    fitx = np.polyfit(t, x, 3)
    fity = np.polyfit(t, y, 3)
    fitz = np.polyfit(t, z, 3)
    xx = np.polyval(fitx, t)
    yy = np.polyval(fity, t)
    zz = np.polyval(fitz, t)
    for i in np.arange(N-1):
      ax.plot3D(xx[i:i+2], yy[i:i+2], zz[i:i+2], lw=line_width, color=get_sequential_color(i, N))

  def GenerateSamples(self, theta, phi):
    return SphericalToCartesian(self.radius, theta, phi)

def QuaternionsToStereographicProjection(a, b, c, d):
  if np.abs(1.0-a).any() < 1e-6:
    #print("WARNING: singularity")
    return [np.nan,np.nan,np.nan]

  kScalar = 1
  x = kScalar * b / (1.0-a);
  y = kScalar * c / (1.0-a);
  z = kScalar * d / (1.0-a);
  return [x,y,z]

class SO3StereographicProjection:
  def __init__(self, ax, x=0.0, y=0.0, z=0.0, radius=1.0):
    self.ax = ax
    self.x_position = x
    self.y_position = y
    self.z_position = z
    self.radius = radius
    self.zorder=1
    self.fiber_number = 0
    self.colormap = plt.cm.jet(np.linspace(0,1,1000))

  def DrawFiber(self, fiber, color, zorder=1):
    points = np.array(fiber['states'])
    try:
      #self.ax.scatter3D(points[:,0], points[:,1], points[:,2], s=kMarkerSize, color=kColorSpherePoint, zorder=zorder)
      circle = Circle3D(points)
      t = np.linspace(0.0, 2 * np.pi, 1000)
      points = circle.equation(t)
      self.ax.plot(points[:, 0], points[:, 1], points[:, 2], color=color, zorder=zorder, lw=kLineWidthHopfFiber)
      self.fiber_number = self.fiber_number + 1

    except Exception as error:
      print('Could not draw circle ' + repr(error))

  def DrawPoint(self, point, zorder=1):
    self.ax.scatter3D(point[0], point[1], point[2], s=kMarkerSize, color=kColorSpherePoint, zorder=zorder)

  def DrawSpecialPoint(self, point, color, markersize=5, zorder=1):
    self.ax.scatter3D(point[0], point[1], point[2], s=markersize, color=color, zorder=zorder)

  def DrawLine(self, p1, p2, color, zorder=1):
    self.ax.plot3D([p1[0],p2[0]], [p1[1],p2[1]], [p1[2],p2[2]], lw=kLineWidth, color=color, zorder=zorder)

  def DrawEdge(self, points, color, zorder=1):
    #print(points)
    #DrawEdgeFromPoints(self.ax, points, kMarkerSize, kLineWidth, color, zorder)
    t = np.arange(points.shape[0])  # simple assumption that data was sampled in regular steps
    x = points[:,0]
    y = points[:,1]
    z = points[:,2]
    self.ax.scatter3D(x[0], y[0], z[0], s=kMarkerSize, color=color, zorder=zorder)
    fitx = np.polyfit(t, x, 3)
    fity = np.polyfit(t, y, 3)
    fitz = np.polyfit(t, z, 3)
    xx = np.polyval(fitx, t)
    yy = np.polyval(fity, t)
    zz = np.polyval(fitz, t)
    self.ax.plot3D(xx, yy, zz, lw=3, color=color, zorder=zorder)

def DrawSphereFromYaml(yaml_node):
  fig = plt.figure(dpi=kDPI)
  ax = fig.add_subplot(projection='3d', computed_zorder=False)

  sphere = Sphere(ax)
  sphere.Draw()

  edges = yaml_node["trees"]["sphere_tree"]["edges"]

  start = yaml_node["trees"]["sphere_tree"]["start"]
  sphere.DrawSpecialPoint(start, color=kColorSphereStartPoint, markersize=kMarkerSizeSpherePoint, zorder=6)
  goal = yaml_node["trees"]["sphere_tree"]["goal"]
  sphere.DrawSpecialPoint(goal, color=kColorSphereGoalPoint, markersize=kMarkerSizeSpherePoint, zorder=6)

  counter = 0
  #color = get_random_color(counter)
  counter = counter+1
  # p2_old = None
  # for edge in edges:
  #   v1 = edge["source"]
  #   v2 = edge["target"]
  #   p1 = np.array(list(v1), dtype='float32')
  #   p2 = np.array(list(v2), dtype='float32')
  #   if p2_old is not None:
  #     print(p2_old, p1)
  #     if np.abs(p1-p2_old).any() > 1e-3:
  #       color = get_random_color(counter)
  #       counter = counter+1
  #       print(counter)
  #   sphere.DrawLine(p1, p2, color)
  #   p2_old = p2

  points = []
  for edge in edges:
    v1 = edge["source"]
    v2 = edge["target"]
    p1 = np.array(list(v1), dtype='float32')
    p2 = np.array(list(v2), dtype='float32')
    if not points:
      points.append(p1)
      points.append(p2)
    else:
      d = np.linalg.norm(np.array(points[-1]) - p1)
      if d <= 1e-3:
        points.append(p1)
        points.append(p2)
      else:
        # points = np.concatenate(points, axis=0).reshape((-1,3))
        # sphere.DrawEdge(points, color=kColorHopfEdge, zorder=5);
        points = []
        points.append(p1)
        points.append(p2)

  points = np.concatenate(points, axis=0).reshape((-1,3))
  sphere.DrawEdge(points, color=kColorMapFiber, zorder=5);

  ax.set_aspect('auto')
  fig.tight_layout()
  kLimit = 1
  ax.set_xlim(np.array([-kLimit, +kLimit]))
  ax.set_ylim(np.array([-kLimit, +kLimit]))
  ax.set_zlim(np.array([-kLimit, +kLimit]))
  ax.axis('off')
  ax.view_init(elev=10., azim=180)
  plt.show()
  plt.savefig("hopffibration_sphere.png", dpi=kDPI)

def GetMaxFibers(fiber_node):
  counter = 0
  for i in range(0,len(fiber_node)):
    fibers = fiber_node[i]
    if fibers is None:
      continue
    for fiber in fibers:
      counter = counter+1
  return counter

def DrawHopfFibrationFromYaml(yaml_node):
  fig = plt.figure(dpi=kDPI)
  ax = fig.add_subplot(projection='3d', computed_zorder=False)
  hopffibration = SO3StereographicProjection(ax)

  start = yaml_node["trees"]["so3_tree"]["start"]
  hopffibration.DrawSpecialPoint(start, color=kColorSphereStartPoint, markersize=kMarkerSizeHopfStartPoint, zorder=6)
  goal = yaml_node["trees"]["so3_tree"]["goal"]
  hopffibration.DrawSpecialPoint(goal,  color=kColorSphereGoalPoint, markersize=kMarkerSizeHopfGoalPoint, zorder=6)

  edges = yaml_node["trees"]["so3_tree"]["edges"]
  # for edge in edges:
  #   v1 = edge["source"]
  #   v2 = edge["target"]
  #   p1 = np.array(list(v1), dtype='float32')
  #   p2 = np.array(list(v2), dtype='float32')
  #   hopffibration.DrawLine(p1, p2, color=kColorHopfEdge, zorder=5)

  ##Collect multiple edges when their target/source match up
  points = []
  for edge in edges:
    v1 = edge["source"]
    v2 = edge["target"]
    p1 = np.array(list(v1), dtype='float32')
    p2 = np.array(list(v2), dtype='float32')
    if not points:
      points.append(p1)
      points.append(p2)
    else:
      d = np.linalg.norm(np.array(points[-1]) - p1)
      if d <= 1e-3:
        points.append(p1)
        points.append(p2)
      else:
        points = np.concatenate(points, axis=0).reshape((-1,3))
        hopffibration.DrawEdge(points, color=kColorHopfEdge, zorder=5);
        points = []
        points.append(p1)
        points.append(p2)
  points = np.concatenate(points, axis=0).reshape((-1,3))
  hopffibration.DrawEdge(points, color=kColorHopfEdge, zorder=5);


  fiber_node = yaml_node["fibers"]
  counter = 0
  n = GetMaxFibers(fiber_node)
  for i in range(0,len(fiber_node)):
    fibers = fiber_node[i]
    if fibers is None:
      continue
    for fiber in fibers:
      color = get_sequential_color(counter, n)
      counter = counter+1
      hopffibration.DrawFiber(fibers[fiber], color, zorder=2)

  ax.set_aspect('auto')
  fig.tight_layout()
  kLimit = 1
  ax.set_xlim(np.array([-kLimit, +kLimit]))
  ax.set_ylim(np.array([-kLimit, +kLimit]))
  ax.set_zlim(np.array([-kLimit, +kLimit]))
  ax.axis('off')
  plt.savefig("hopffibration_stereographic_projection.svg", format="svg", dpi=300)
  plt.show()


if __name__ == "__main__":
  parser = argparse.ArgumentParser(
      prog="PlotHopfFibrationSearchTree",
      description="A script to visualize a search tree on a hopf fibration.",
      epilog='')
  parser.add_argument('filename')

  args = parser.parse_args()
  filename = args.filename

  with open(filename, 'r') as file:
    yaml_node = yaml.safe_load(file)
    #DrawSphereFromYaml(yaml_node)
    DrawHopfFibrationFromYaml(yaml_node)

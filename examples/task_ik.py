
import numpy as np
import sys
import pinocchio
import meshcat
import hppfcl

#import qpsolvers
from scipy.spatial.transform import Rotation

#import meshcat_shapes
#import meshcat.geometry as mg

import pink
from pink import solve_ik
from pink.tasks import FrameTask, JointCouplingTask, PostureTask

class AtlasRobotWorld:

  def DisplayPoint(self, position):
    print(position[0])
    print(position[1])
    print(position[2])
    pose = pinocchio.SE3(np.eye(3), np.array([position[0], position[1], position[2]]))
    object = pinocchio.GeometryObject.CreateCapsule(0, 0)
    object.name = "point"
    object.disableCollision = True
    object.parentJoint = 0 #universe is 0
    object.geometry=hppfcl.Sphere(0.1)
    object.placement = pose
    object.meshColor = np.array([1,1,0,1])
    self.viz.addGeometryObject(object)

  def DisplayContact(self, pose):
    x = pose.translation[0]
    y = pose.translation[1]
    z = pose.translation[2]
    pose.translation[2] = pose.translation[2] -0.5*pose.translation[2]
    #pose = pinocchio.SE3(np.eye(3), np.array([x, y, z]))
    object = pinocchio.GeometryObject.CreateCapsule(0, 0)
    object.name = "contact"+str(self.counter)
    self.counter = self.counter+1
    object.disableCollision = True
    object.parentJoint = 0 #universe is 0
    object.geometry=hppfcl.Cone(0.2, 1)
    object.placement = pose
    object.meshColor = np.array([1,1,0,1])
    self.viz.addGeometryObject(object)

  def __init__(self):
    self.counter = 0
    self.robot = pinocchio.RobotWrapper.BuildFromURDF(
      filename="/home/aorthey/git/pink/data/atlas.urdf",
      package_dirs="/home/aorthey/git/pink/data/",
      root_joint=pinocchio.JointModelFreeFlyer(),
    )

    #############################################
    #Add obstacle
    #############################################
    x = +1.0
    y = +0.0
    z = +1.0
    pose = pinocchio.SE3(np.eye(3), np.array([x, y, z]))
    object = pinocchio.GeometryObject.CreateCapsule(0, 0)
    object.name = "box"
    object.disableCollision = False
    object.parentJoint = 0 #universe is 0
    object.geometry=hppfcl.Box(1, 1, 1)
    object.placement = pose
    object.meshColor = np.array([1,0,0,1])
    self.robot.collision_model.addGeometryObject(object, self.robot.model)
    self.robot.visual_model.addGeometryObject(object, self.robot.model)
    #############################################

    self.robot.collision_model.addAllCollisionPairs()
    self.robot.collision_data = pinocchio.GeometryData(self.robot.collision_model)

    self.configuration = pink.Configuration(self.robot.model, self.robot.data, self.robot.q0)
    self.viz = pinocchio.visualize.MeshcatVisualizer(
      self.robot.model, self.robot.collision_model, self.robot.visual_model
    )
    self.robot.setVisualizer(self.viz, init=False)
    self.viz.initViewer(open=True)
    self.viz.loadViewerModel()
    self.viz.display(self.configuration.q)

    for k in range(0,40):
      removeable_indices = []
      if self.IsInCollision(self.configuration):
        for i,p in enumerate(self.robot.collision_model.collisionPairs):
          if pinocchio.computeCollision(self.robot.collision_model,self.robot.collision_data,i):
            removeable_indices.append(i)

      for idx in removeable_indices:
        print("Remove collision pair: ",idx)
        if idx < len(self.robot.collision_model.collisionPairs):
          self.robot.collision_model.removeCollisionPair(self.robot.collision_model.collisionPairs[idx])

      self.robot.collision_data = pinocchio.GeometryData(self.robot.collision_model)

  def IsInCollision(self, configuration, verbose=False):
    is_in_collision = pinocchio.computeCollisions( \
        self.robot.model, \
        self.robot.data, \
        self.robot.collision_model, \
        self.robot.collision_data,  \
        configuration.q, False)

    if verbose:
      print(80*"#")
      print("Is in collision:", ("Yes" if is_in_collision else "No"))

      # # Print the status of collision for all collision pairs
      for i,p in enumerate(self.robot.collision_model.collisionPairs):
        if pinocchio.computeCollision(self.robot.collision_model,self.robot.collision_data,i):
            print(i,p, self.robot.collision_model.geometryObjects[p.first].name, \
                self.robot.collision_model.geometryObjects[p.second].name)

    return is_in_collision


  def SolveIK(self, translation_left_foot, translation_right_foot):

    ## SEED
    configuration = pink.Configuration(self.robot.model, self.robot.data, self.robot.q0)

    ## Tasks initialization for IK
    left_foot_task = FrameTask(
      "l_foot",
      position_cost=1.0,
      orientation_cost=1.0,
    )
    right_foot_task = FrameTask(
      "r_foot",
      position_cost=1.0,
      orientation_cost=1.0,
    )

    tasks = [
      left_foot_task,
      right_foot_task,
    ]
    rotation = Rotation.from_euler('xyz', [-0.1*np.pi, -0.1*np.pi, 0.0], degrees=False)
    R = np.array(rotation.as_matrix())
    transform_l_ankle_target_to_init = pinocchio.SE3(
      R, translation_left_foot
    )
    transform_r_ankle_target_to_init = pinocchio.SE3(
      np.eye(3), translation_right_foot
    )
    left_foot_task.set_target(
      configuration.get_transform_frame_to_world("l_foot")
      * transform_l_ankle_target_to_init
    )
    right_foot_task.set_target(
      configuration.get_transform_frame_to_world("r_foot")
      * transform_r_ankle_target_to_init
    )

    dt = 1e-2
    damping = 1e-8
    niter = 1000

    for i in range(niter):
      dv = pink.solve_ik(
          configuration,
          tasks,
          dt=dt,
          damping=damping,
          solver="quadprog",
      )
      q_out = pinocchio.integrate(self.robot.model, configuration.q, dv * dt)
      configuration = pink.Configuration(self.robot.model, self.robot.data, q_out)
      pinocchio.updateFramePlacements(self.robot.model, self.robot.data)
      err = 0.0
      for task in tasks:
        err += task.compute_error(configuration)
      print(i, err)
      if np.linalg.norm(err) < 1e-8:
          break

    self.DisplayContact(transform_r_ankle_target_to_init)
    self.DisplayContact(transform_l_ankle_target_to_init)

    print("Found solution: ", configuration.q)
    return configuration

  def Visualize(self, configuration):
    #############################################
    self.viz.display(configuration.q)

  def IsStaticallyStable(self, configuration):
    pinocchio.forwardKinematics(self.robot.model, self.robot.data, configuration.q)
    pinocchio.updateFramePlacements(self.robot.model, self.robot.data)
    com = pinocchio.centerOfMass(self.robot.model, self.robot.data)
    print(com)

    extForce = np.array([0., .0, -9.81]) # units are N
    extCentroidalTorque = np.array([.0, .0, .0]) # units are Nm
    extCentroidalWrench = np.hstack([extForce, extCentroidalTorque])

    return com

    ################################################################
    ### Compute static stability
    ################################################################
    ##https://github.com/mfocchi/jet-leg/blob/b3607a7cb083d805467bae657080a8f5101dc89f/examples/examples_go1/feasible_wrench_polytope.py#L112
    ## jet_leg python package

    ##https://github.com/robot-descriptions/awesome-robot-descriptions

    #sys.exit(0)
    ################################################################


world = AtlasRobotWorld()
translation_left_foot = np.array([0.5, 0.3, 1.6])
translation_right_foot = np.array([-0.4, 0.0, 1.0])
q = world.SolveIK(translation_left_foot, translation_right_foot)
com=world.IsStaticallyStable(q)
world.DisplayPoint(com)
world.IsInCollision(q, True)
world.Visualize(q)


###TODO:
## [ ] Visualize cones at correct contact
## [ ] Visualize COM
## [ ] Check for static stability (maybe visualize wrench cones)
## [ ] Integrate with OMPL as FootStepStateSpace or something

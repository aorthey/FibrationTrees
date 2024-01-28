import numpy as np
from ompl import base
from ompl import geometric
from functools import partial

def double_pendulum_steering(state, goal_state, epsilon=0.01):
  g = 9.81
  m1 = 1
  m2 = 1
  l1 = 1
  l2 = 1
  # Get the current joint angles
  theta1 = state[0]
  theta2 = state[1]

  # Get the desired joint angles
  theta1_desired = goal_state[0]
  theta2_desired = goal_state[1]

  # Compute the desired angular velocities
  dtheta1_desired = (theta1_desired - theta1) / epsilon
  dtheta2_desired = (theta2_desired - theta2) / epsilon

  # Compute the desired torques
  torque1 = 2 * (g * m1 * np.sin(theta1) + m2 * (l2 * (dtheta2_desired**2) * \
    np.sin(theta1 - theta2) + g * np.sin(theta1 - theta2))) / (l1 * (m1 + m2))
  torque2 = -(2 * (m2 * l2 * (dtheta2_desired**2) * np.sin(theta2) + g * \
    np.sin(theta2)) + m1 * l1 * (dtheta1_desired**2) * np.sin(theta2 - theta1)) / (l2 * (m1 + m2))

  return np.array([torque1, torque2])

class velocityMotionValidator(base.MotionValidator):
  def checkMotion(self, s1, s2):
    tmax = 10
    [t1, t2] = double_pendulum_steering(s1, s2)
    #print(t1, t2)
    return np.abs(t1) <= tmax and np.abs(t2) <= tmax

def isStateValid(spaceInformation, state):
  x = state[0]
  if (x > np.pi/2 - np.pi/8) and (x < np.pi/2 + np.pi/8):
    return False
  return True

space = base.RealVectorStateSpace(4)
bounds = base.RealVectorBounds(4)
bounds.setLow(0, -np.pi)
bounds.setHigh(0, +np.pi)
bounds.setLow(1, -np.pi)
bounds.setHigh(1, +np.pi)
bounds.setLow(2, -10)
bounds.setHigh(2, +10)
bounds.setLow(3, -10)
bounds.setHigh(3, +10)
space.setBounds(bounds)

si = base.SpaceInformation(space)
si.setMotionValidator(velocityMotionValidator(si))
si.setStateValidityChecker(base.StateValidityCheckerFn( \
    partial(isStateValid, si)))

################################################################################
# Create a problem instance
################################################################################
start = base.State(space)
goal = base.State(space)

start[0]=0
start[1]=0
start[2]=0
start[3]=0
goal[0]=np.pi
goal[1]=0
goal[2]=0
goal[3]=0

problem = base.ProblemDefinition(si)
problem.setStartAndGoalStates(start, goal)

################################################################################
# Solve problem
################################################################################
planner = geometric.RRTConnect(si)
planner.setProblemDefinition(problem)
result = planner.solve(10.0)
if not result:
  print("No solution found.")
  sys.exit(1)

path = problem.getSolutionPath()
print("Found path of length", path.length())
print("Path:", path.printAsMatrix())

states = path.getStates()
for k in range(1, len(states)):
  s1 = states[k-1]
  s2 = states[k]
  [t1, t2] = double_pendulum_steering(s1, s2)

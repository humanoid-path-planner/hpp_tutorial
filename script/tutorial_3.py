# Import libraries and load robots. {{{1

# Import. {{{2
from hpp.gepetto import PathPlayer
from hpp.corbaserver.manipulation.pr2 import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, Rule, \
  Constraints, ConstraintGraphFactory, Client
from hpp.gepetto.manipulation import ViewerFactory
from hpp.corbaserver import loadServerPlugin
loadServerPlugin ("corbaserver", "manipulation-corba.so")
Client ().problem.resetProblem ()
# 2}}}

# Load PR2 and a box to be manipulated. {{{2
class Box (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_tutorial'
  meshPackageName = 'hpp_tutorial'
  urdfName = 'box'
  urdfSuffix = ""
  srdfSuffix = ""

class Environment (object):
  packageName = 'iai_maps'
  meshPackageName = 'iai_maps'
  urdfName = 'kitchen_area'
  urdfSuffix = ""
  srdfSuffix = ""

robot = Robot ('pr2-box', 'pr2')
ps = ProblemSolver (robot)
## ViewerFactory is a class that generates Viewer on the go. It means you can
## restart the server and regenerate a new windows.
## To generate a window:
## vf.createViewer ()
vf = ViewerFactory (ps)

vf.loadObjectModel (Box, 'box')
vf.loadEnvironmentModel (Environment, "kitchen_area")

robot.setJointBounds ("pr2/root_joint", [-5,-2,-5.2,-2.7]     )
robot.setJointBounds ("box/root_joint", [-5.1,-2,-5.2,-2.7,0,1.5])
# 2}}}

# 1}}}

# Initialization. {{{1

# Set parameters. {{{2
# robot.client.basic.problem.resetRoadmap ()
ps.setErrorThreshold (1e-3)
ps.setMaxIterProjection (40)
# 2}}}

# Generate initial and goal configuration. {{{2
q_init = robot.getCurrentConfig ()
rank = robot.rankInConfiguration ['pr2/l_gripper_l_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/l_gripper_r_finger_joint']
q_init [rank] = 0.5
q_init [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['pr2/torso_lift_joint']
q_init [rank] = 0.2
rank = robot.rankInConfiguration ['box/root_joint']
q_init [rank:rank+3] = [-2.5, -4, 0.8]

q_goal = q_init [::]
q_goal [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['box/root_joint']
q_goal [rank:rank+3] = [-3.5, -4, 0.8]
# 2}}}

# Create the constraints. {{{2
locklhand = ['l_l_finger','l_r_finger'];
ps.createLockedJoint ('l_l_finger', 'pr2/l_gripper_l_finger_joint', [0.5,])
ps.createLockedJoint ('l_r_finger', 'pr2/l_gripper_r_finger_joint', [0.5,])
# 2}}}

# Create the constraint graph. {{{2
# Define the set of grippers used for manipulation
grippers = [ "pr2/l_gripper", ]
# Define the set of objects that can be manipulated
objects = [ "box", ]
# Define the set of handles for each object
handlesPerObject = [ ["box/handle2", ], ]
# Define the set of contact surfaces used for each object
contactSurfacesPerObject = [ ["box/box_surface", ], ]
# Define the set of contact surfaces of the environment used to put objects
envContactSurfaces = [ "kitchen_area/pancake_table_table_top", ]
# Define rules for associating grippers and handles (here all associations are
# allowed)
rules = [ Rule([".*"], [".*"], True), ]

cg = ConstraintGraph (robot, 'graph')
factory = ConstraintGraphFactory (cg)
factory.setGrippers (grippers)
factory.environmentContacts (envContactSurfaces)
factory.setObjects (objects, handlesPerObject, contactSurfacesPerObject)
factory.setRules (rules)
factory.generate ()
cg.addConstraints (graph = True, constraints = Constraints \
                   (lockedJoints = locklhand))
cg.initialize ()

# 2}}}

res, q_init_proj, err = cg.applyNodeConstraints("free", q_init)
res, q_goal_proj, err = cg.applyNodeConstraints("free", q_goal)

ps.setInitialConfig (q_init_proj)

ps.addGoalConfig (q_goal_proj)

# print ps.solve()

# ps.setTargetState (cg.nodes["pr2/l_gripper grasps box/handle2"])
# print ps.solve()

# 1}}}

# v = vf.createViewer ()
# v (q_init_proj)
# pp = PathPlayer (v)

# vim: foldmethod=marker foldlevel=1

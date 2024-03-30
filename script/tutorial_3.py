# Import libraries and load robots. {{{1

# Import.
from math import sqrt

from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import (
    Client,
    ConstraintGraph,
    ConstraintGraphFactory,
    Constraints,
    ProblemSolver,
    Rule,
)
from hpp.corbaserver.manipulation.pr2 import Robot
from hpp.gepetto import PathPlayer  # noqa: F401
from hpp.gepetto.manipulation import ViewerFactory

loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()

# Load PR2 and a box to be manipulated.


class Box:
    rootJointType = "freeflyer"
    packageName = "hpp_tutorial"
    urdfName = "box"
    urdfSuffix = ""
    srdfSuffix = ""


class Environment:
    packageName = "hpp_tutorial"
    urdfName = "kitchen_area"
    urdfSuffix = ""
    srdfSuffix = ""


robot = Robot("pr2-box", "pr2")
ps = ProblemSolver(robot)
# ViewerFactory is a class that generates Viewer on the go. It means you can
# restart the server and regenerate a new windows.
# To generate a window:
# vf.createViewer ()
vf = ViewerFactory(ps)

vf.loadObjectModel(Box, "box")
vf.loadEnvironmentModel(Environment, "kitchen_area")

robot.setJointBounds("pr2/root_joint", [-5, -2, -5.2, -2.7])
robot.setJointBounds("box/root_joint", [-5.1, -2, -5.2, -2.7, 0, 1.5])

# Initialization.

# Set parameters.
# robot.client.basic.problem.resetRoadmap ()
ps.setErrorThreshold(1e-3)
ps.setMaxIterProjection(40)

# Generate initial and goal configuration.
q_init = robot.getCurrentConfig()
rank = robot.rankInConfiguration["pr2/l_gripper_l_finger_joint"]
q_init[rank] = 0.5
rank = robot.rankInConfiguration["pr2/l_gripper_r_finger_joint"]
q_init[rank] = 0.5
q_init[0:2] = [-3.2, -4]
rank = robot.rankInConfiguration["pr2/torso_lift_joint"]
q_init[rank] = 0.2
rank = robot.rankInConfiguration["box/root_joint"]
q_init[rank : rank + 3] = [-2.5, -4, 0.746]

# Put box in right orientation
q_init[rank + 3 : rank + 7] = [0, -sqrt(2) / 2, 0, sqrt(2) / 2]

q_goal = q_init[::]
q_goal[0:2] = [-3.2, -4]
rank = robot.rankInConfiguration["box/root_joint"]
q_goal[rank : rank + 3] = [-2.5, -4.5, 0.746]

# Create the constraints.
locklhand = ["l_l_finger", "l_r_finger"]
ps.createLockedJoint(
    "l_l_finger",
    "pr2/l_gripper_l_finger_joint",
    [
        0.5,
    ],
)
ps.createLockedJoint(
    "l_r_finger",
    "pr2/l_gripper_r_finger_joint",
    [
        0.5,
    ],
)

# Create the constraint graph.
# Define the set of grippers used for manipulation
grippers = [
    "pr2/l_gripper",
]
# Define the set of objects that can be manipulated
objects = [
    "box",
]
# Define the set of handles for each object
handlesPerObject = [
    [
        "box/handle2",
    ],
]
# Define the set of contact surfaces used for each object
contactSurfacesPerObject = [
    [
        "box/box_surface",
    ],
]
# Define the set of contact surfaces of the environment used to put objects
envContactSurfaces = [
    "kitchen_area/pancake_table_table_top",
]
# Define rules for associating grippers and handles (here all associations are
# allowed)
rules = [
    Rule([".*"], [".*"], True),
]

cg = ConstraintGraph(robot, "graph")
factory = ConstraintGraphFactory(cg)
factory.setGrippers(grippers)
factory.environmentContacts(envContactSurfaces)
factory.setObjects(objects, handlesPerObject, contactSurfacesPerObject)
factory.setRules(rules)
factory.generate()
cg.addConstraints(graph=True, constraints=Constraints(numConstraints=locklhand))
cg.initialize()

ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)

# uncomment to solve
# ps.solve()

# Path optimization uncomment to optimize
#
# ps.loadPlugin('manipulation-spline-gradient-based.so')
# ps.addPathOptimizer('SplineGradientBased_bezier1')
# ps.optimizePath(0)

# display in gepetto-gui
# v = vf.createViewer ()

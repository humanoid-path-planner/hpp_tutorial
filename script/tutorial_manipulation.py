# Import libraries and load robots. {{{1

# Import. {{{2
from hpp.corbaserver.manipulation.pr2 import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, Rule
from hpp.gepetto.manipulation import ViewerFactory
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
## fk.createRealClient ()
fk = ViewerFactory (ps)

fk.loadObjectModel (Box, 'box')
fk.loadEnvironmentModel (Environment, "kitchen_area")

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

graph = ConstraintGraph.buildGenericGraph (robot, "manipulate_box",
        [ "pr2/l_gripper", ],
        [ "box", ],
        [ ["box/handle2", ], ],
        [ ["box/box_surface", ], ],
        [ "kitchen_area/pancake_table_table_top", ],
        [ Rule([".*"], [".*"], True), ]
        )
        # Allow everything
graph.setConstraints (graph = True, lockDof = locklhand)

# 2}}}

res, q_init_proj, err = graph.applyNodeConstraints("free", q_init)
res, q_goal_proj, err = graph.applyNodeConstraints("free", q_goal)

ps.setInitialConfig (q_init_proj)

ps.addGoalConfig (q_goal_proj)
print ps.solve()

# 1}}}

#v = fk.createRealClient ()
#pp = PathPlayer (robot.client.basic, v)

# vim: foldmethod=marker foldlevel=1

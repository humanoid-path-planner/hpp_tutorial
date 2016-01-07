# Import libraries and load robots. {{{1

# Import. {{{2
from hpp.corbaserver.manipulation.pr2 import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph
from hpp.gepetto.manipulation import Viewer, ViewerFactory
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

robot.setJointBounds ("pr2/base_joint_xy" , [-5,-2,-5.2,-2.7]     )
robot.setJointBounds ("box/base_joint_xyz", [-5.1,-2,-5.2,-2.7,0,1.5])
# 2}}}

# Load the Python class ConstraintGraph. {{{2
graph = ConstraintGraph (robot, 'graph')
# 2}}}

# 1}}}

# Initialization. {{{1

# Set parameters. {{{2
robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterations (40)
# 2}}}

# Create lists of joint names - useful to define passive joints. {{{2
jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['pr2'] = list ()
jointNames['allButPR2LeftArm'] = list ()
for n in jointNames['all']:
  if n.startswith ("pr2"):
    jointNames['pr2'].append (n)
  if not n.startswith ("pr2/l_gripper"):
    jointNames['allButPR2LeftArm'].append (n)

ps.addPassiveDofs ('pr2', jointNames['pr2'])
# 2}}}

# Generate initial and goal configuration. {{{2
q_init = robot.getCurrentConfig ()
rank = robot.rankInConfiguration ['pr2/l_gripper_l_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/l_gripper_r_finger_joint']
q_init [rank] = 0.5
q_goal = q_init [::]
q_init [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['pr2/torso_lift_joint']
q_init [rank] = 0.2
rank = robot.rankInConfiguration ['box/base_joint_xyz']
q_init [rank:rank+3] = [-2.5, -4, 0.8]

q_goal [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['pr2/l_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/l_elbow_flex_joint']
q_goal [rank] = -0.5
rank = robot.rankInConfiguration ['pr2/r_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/r_elbow_flex_joint']
q_goal [rank] = -0.5
rank = robot.rankInConfiguration ['box/base_joint_xyz']
q_goal [rank:rank+3] = [-4.8, -4.6, 0.9]
rank = robot.rankInConfiguration ['box/base_joint_SO3']
q_goal [rank:rank+4] = [0, 0, 0, 1]
# 2}}}

# Create the constraints. {{{2
graph.createGrasp ('l_grasp', 'pr2/l_gripper', 'box/handle', passiveJoints = 'pr2')
graph.createPreGrasp ('l_pregrasp', 'pr2/l_gripper', 'box/handle')

lockbox = ps.lockFreeFlyerJoint ('box/base_joint', 'box_lock')
lockpr2 = ps.lockPlanarJoint ('pr2/base_joint', 'pr2_lock')
lockboth = lockpr2[:]; lockboth.extend (lockbox)

locklhand = ['l_l_finger','l_r_finger'];
ps.createLockedJoint ('l_l_finger', 'pr2/l_gripper_l_finger_joint', [0.5,])
ps.createLockedJoint ('l_r_finger', 'pr2/l_gripper_r_finger_joint', [0.5,])
# 2}}}

# Create the constraint graph. {{{2

# Create nodes and edges. {{{3
graph.createNode (['box', 'free'])

we = dict ()
we["ungrasp"] = graph.createWaypointEdge ('box', 'free', "ungrasp", nb=1, weight=1)
we[  "grasp"] = graph.createWaypointEdge ('free', 'box',   "grasp", nb=1, weight=10)

graph.createEdge ('free', 'free', 'move_free', 1)
graph.createEdge ('box', 'box', 'keep_grasp', 5)
graph.createLevelSetEdge ('box', 'box', 'keep_grasp_ls', 10)
# 3}}}

# Set the constraints of the component of the graph. {{{3
graph.setConstraints (node='box', grasp='l_grasp')
graph.setConstraints (edge='move_free', lockDof = lockbox)
graph.setConstraints (edge="ungrasp_e1", lockDof = lockbox)
graph.setConstraints (node="ungrasp_n0", pregrasp = 'l_pregrasp')
#graph.setConstraints (edge="ungrasp_e0", lockDof = lockboth)
graph.setConstraints (edge="ungrasp_e0", lockDof = lockbox)
#graph.setConstraints (edge="grasp_e1", lockDof = lockboth)
graph.setConstraints (edge="grasp_e1", lockDof = lockbox)
graph.setConstraints (node="grasp_n0", pregrasp = 'l_pregrasp')
graph.setConstraints (edge="grasp_e0", lockDof = lockbox)
#graph.client.graph.setLevelSetConstraints  (graph.edges["keep_grasp_ls"], [], lockbox)
graph.setLevelSetConstraints ("keep_grasp_ls", lockDof = lockbox)
graph.setConstraints (graph = True, lockDof = locklhand)
# 3}}}

# 2}}}

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# 1}}}

# vim: foldmethod=marker foldlevel=1

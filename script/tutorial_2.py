from hpp.corbaserver.pr2 import Robot
robot = Robot ('pr2')
robot.setJointBounds ("root_joint", [-4, -3, -5, -3])

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['torso_lift_joint']
q_init [rank] = 0.2
vf (q_init)

q_goal [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['l_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['l_elbow_flex_joint']
q_goal [rank] = -0.5
rank = robot.rankInConfiguration ['r_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['r_elbow_flex_joint']
q_goal [rank] = -0.5
vf (q_goal)

vf.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.selectPathPlanner ("PRM")
ps.addPathOptimizer ("RandomShortcut")

# print (ps.solve ())

## Uncomment this to connect to a viewer server and play solution paths
#
# v = vf.createViewer()
# from hpp.gepetto import PathPlayer
# pp = PathPlayer (robot.client, v)

# pp (0)
# pp (1)

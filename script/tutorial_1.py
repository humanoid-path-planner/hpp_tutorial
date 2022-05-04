from hpp.corbaserver.pr2 import Robot
from hpp.corbaserver import ProblemSolver
from hpp.gepetto import ViewerFactory

robot = Robot("pr2")
robot.setJointBounds("root_joint", [-4, -3, -5, -3])

ps = ProblemSolver(robot)

vf = ViewerFactory(ps)

q_init = robot.getCurrentConfig()
q_goal = q_init[::]
q_init[0:2] = [-3.2, -4]
rank = robot.rankInConfiguration["torso_lift_joint"]
q_init[rank] = 0.2

q_goal[0:2] = [-3.2, -4]
rank = robot.rankInConfiguration["l_shoulder_lift_joint"]
q_goal[rank] = 0.5
rank = robot.rankInConfiguration["l_elbow_flex_joint"]
q_goal[rank] = -0.5
rank = robot.rankInConfiguration["r_shoulder_lift_joint"]
q_goal[rank] = 0.5
rank = robot.rankInConfiguration["r_elbow_flex_joint"]
q_goal[rank] = -0.5

vf.loadObstacleModel("package://hpp_tutorial/urdf/kitchen_area.urdf", "kitchen")

ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)

ps.addPathOptimizer("RandomShortcut")

# print (ps.solve ())

# # Uncomment this to connect to a viewer server and play solution paths
#
# v = vf.createViewer()
# from hpp.gepetto import PathPlayer
# pp = PathPlayer (v)

# pp (0)
# pp (1)

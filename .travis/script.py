from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot

robot = Robot("pr2")
robot.setJointBounds("base_joint_xy", [-4, -3, -5, -3])


ps = ProblemSolver(robot)

# On travis, we do not compile the viewer (yet?)
# from hpp.gepetto import ViewerFactory
# r = ViewerFactory (ps)

q_init = robot.getCurrentConfig()
q_goal = q_init[::]
q_init[0:2] = [-3.2, -4]
rank = robot.rankInConfiguration["torso_lift_joint"]
q_init[rank] = 0.2
# r (q_init)

q_goal[0:2] = [-3.2, -4]
rank = robot.rankInConfiguration["l_shoulder_lift_joint"]
q_goal[rank] = 0.5
rank = robot.rankInConfiguration["l_elbow_flex_joint"]
q_goal[rank] = -0.5
rank = robot.rankInConfiguration["r_shoulder_lift_joint"]
q_goal[rank] = 0.5
rank = robot.rankInConfiguration["r_elbow_flex_joint"]
q_goal[rank] = -0.5
# r (q_goal)

# r.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")
ps.loadObstacleFromUrdf("iai_maps", "kitchen_area", "kitchen/")

ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)

print(ps.solve())

ps.addPathOptimizer("RandomShortcut")

print(ps.optimizePath(0))

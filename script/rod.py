from hpp.corbaserver.rod import Robot
robot = Robot ('rod')
robot.setJointBounds ("base_joint_xy", [-3, 3, -2, 2])

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer
r = Viewer (ps)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q1 = q_init [::]
q_init [0] = -.5
q_goal [0] = .5
r (q_init)
r (q_goal)
q1 [:2] = (0.,.5)
r.loadObstacleModel ("hpp_tutorial", "box", "box-1")

ps.selectPathValidation ("Dichotomy", 0.)
ps.setInitialConfig (q_init)
ps.addGoalConfig (q1)
ps.solve ()
ps.resetGoalConfigs ()
ps.addGoalConfig (q_goal)
ps.solve ()

ps.addPathOptimizer ("GradientBased")
#ps.optimizePath (ps.numberPaths () - 1)

from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, r)

#pp (0)
#pp (1)

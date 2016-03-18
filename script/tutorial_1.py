from hpp.corbaserver.hpp_dlr_ipa import Robot
robot = Robot ('ipa')

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer
r = Viewer (ps)

r.loadObstacleModel ("hpp-dlr-ipa", 'door', 'Door')

q_door = (0.5,1.6,0.8,1,0,0,0)
ps.getObstacleNames(True, False)
r.moveObstacle ("Door/door_frame_0", q_door)

q_init = robot.getCurrentConfig ()
r(q_init)

q_goal = [0.934, -3.266, -1.212,-0.823, -1.488, 0.269]
r(q_goal)

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.client.problem.getAvailable('PathValidation')
ps.selectPathValidation ("Dichotomy", 0.)

ps.solve()

from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, r)

pp(0)

ps.client.problem.getAvailable('PathOptimizer')
ps.addPathOptimizer ("RandomShortcut")

ps.numberPaths()
ps.optimizePath(0)
ps.numberPaths()

pp(1)

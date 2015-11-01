from hpp.corbaserver.dlr_miiwa import Robot
robot = Robot ('dlr')
robot.setJointBounds ("miiwa_joint_x", [-4, -3,])
robot.setJointBounds ("miiwa_joint_y", [-6.5, -3,])

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer
r = Viewer (ps)

r.loadObstacleModel ("hpp_tutorial", "box", 'box')
r.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")

# Move box on the table
q_box = (-2.5, -4.0, 0.7555686333723849, 0.7071067811865475, 0.,
         0.7071067811865475, 0.)
r.moveObstacle ("box/base_link_0", q_box)

q_init = robot.getCurrentConfig ()
q_init [0:2] = [-3.5, -6]
q_init [2:4] = [0,1]
rank = robot.rankInConfiguration ['schunk_wsg50_joint_left_jaw']
q_init [rank:rank+2] = [0.05, 0.05]
r (q_init)

q_goal = (-3.3668608663093553, -3.8721030610816953, 0.4203599258539973,
          -0.9073574448562275, 2.596600874869132, -1.4551576036136797,
          -2.594171477219449, -1.3380271850818217, 2.1077576346524514,
          -0.645492511677414, -3.03775218971459, 0.05, 0.05,
          0.5157503047048388, 1.261014876256842)
r (q_goal)

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.selectPathValidation ("Dichotomy", 0.)
ps.addPathOptimizer ("RandomShortcut")
ps.selectPathPlanner ("PRM")

ps.solve ()

from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, r)

#pp (0)
#pp (1)

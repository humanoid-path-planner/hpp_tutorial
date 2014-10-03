from hpp.corbaserver.pr2 import Robot
robot = Robot ('pr2')
robot.setJointBounds ("base_joint_xy", [-4, -3, -5, -3])

from hpp_ros import ScenePublisher
r = ScenePublisher (robot)

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['torso_lift_joint']
q_init [rank] = 0.2
r (q_init)

q_goal [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['l_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['l_elbow_flex_joint']
q_goal [rank] = -0.5
rank = robot.rankInConfiguration ['r_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['r_elbow_flex_joint']
q_goal [rank] = -0.5
r (q_goal)

ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area", "")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.solve ()

from hpp_ros import PathPlayer
pp = PathPlayer (robot.client, r)

pp (0)
pp (1)

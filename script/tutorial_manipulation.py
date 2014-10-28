from hpp.corbaserver.manipulation.pr2 import Robot

robot = Robot ('pr2')
robot.loadObjectModel ('box', 'freeflyer', 'hpp_tutorial', 'box', '', '')
robot.buildCompositeRobot ('pr2-box', ['pr2', 'box'])
robot.client.basic.obstacle.loadObstacleModel ("iai_maps", "kitchen_area", "")

robot.setJointBounds ("pr2/base_joint_xy" , [-5,-2,-5.2,-2.7]     )
robot.setJointBounds ("box/base_joint_xyz", [-5.1,-2,-5.2,-2.7,0,1.5])

from hpp_ros.manipulation import ScenePublisher
r = ScenePublisher (robot)

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
r (q_init)

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
r (q_goal)

robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.selectPathOptimizer ('None')
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterations (40)

from hpp.corbaserver.manipulation import ProblemSolver
p = ProblemSolver (robot)

p.createGrasp ('l_grasp', 'pr2/l_gripper', 'box/handle')
p.createGrasp ('l_grasp_passive', 'pr2/l_gripper', 'box/handle')

p.createPreGrasp ('l_pregrasp', 'pr2/l_gripper', 'box/handle')

rankcfg = 0; rankvel = 0; lockbox = list ();
for axis in ['x','y','z']:
  p.createLockedDofConstraint ('box_lock_'  + axis, 'box/base_joint_xyz', 0, rankcfg, rankvel)
  p.createLockedDofConstraint ('box_lock_r' + axis, 'box/base_joint_SO3', 0, rankcfg + 1, rankvel)
  p.isLockedDofParametric ('box_lock_'  + axis ,True)
  p.isLockedDofParametric ('box_lock_r' + axis ,True)
  lockbox.append ('box_lock_'  + axis)
  lockbox.append ('box_lock_r' + axis)
  rankcfg = rankcfg + 1
  rankvel = rankvel + 1

locklhand = ['l_l_finger','l_r_finger'];
p.createLockedDofConstraint ('l_l_finger' , 'pr2/l_gripper_l_finger_joint', 0.5, 0, 0)
p.createLockedDofConstraint ('l_r_finger', 'pr2/l_gripper_r_finger_joint', 0.5, 0, 0)

jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['pr2'] = list ()
jointNames['allButPR2LeftArm'] = list ()
for n in jointNames['all']:
  if n.startswith ("pr2"):
    jointNames['pr2'].append (n)
  if not n.startswith ("pr2/l_gripper"):
    jointNames['allButPR2LeftArm'].append (n)
robot.client.basic.problem.setPassiveDofs ('l_grasp_passive', jointNames['pr2'])

graph = robot.client.manipulation.graph
id = dict()
id["graph"   ] = graph.createGraph ('pr2-box')
id["subgraph"] = graph.createSubGraph ('lefthand')

id["box" ] = graph.createNode (id["subgraph"], 'box')
id["free"] = graph.createNode (id["subgraph"], 'free')

id["ungrasp"] = graph.createWaypointEdge (id["box"], id["free"], "ungrasp", 1, False)
id["ungrasp_w"], id["ungrasp_w_node"] = graph.getWaypoint (id["ungrasp"])

id[  "grasp"] = graph.createWaypointEdge (id["free"], id["box"],   "grasp", 10, True)
id["grasp_w"], id["grasp_w_node"] = graph.getWaypoint (id["grasp"])

id["move_free" ] = graph.createEdge (id["free"    ], id["free"    ], "move_free" , 1 , False)

id["keep_grasp"] = graph.createEdge (id["box"], id["box"], "keep_grasp", 5, False)
id["keep_grasp_ls"] = graph.createLevelSetEdge (id["box"], id["box"], "keep_grasp_ls", 10, False)

graph.setNumericalConstraints (id["box"], ['l_grasp'])
graph.setNumericalConstraintsForPath (id["box"], ['l_grasp_passive'])
graph.setLockedDofConstraints (id["move_free"], lockbox)
graph.setLockedDofConstraints (id["ungrasp"], lockbox)
graph.setNumericalConstraints (id["ungrasp_w_node"], ['l_pregrasp', 'l_pregrasp/ineq_0', 'l_pregrasp/ineq_0.1'])
graph.setLockedDofConstraints (id["ungrasp_w"], lockbox)
graph.setLockedDofConstraints (id["grasp"], lockbox)
graph.setNumericalConstraints (id["grasp_w_node"], ['l_pregrasp', 'l_pregrasp/ineq_0', 'l_pregrasp/ineq_0.1'])
graph.setLockedDofConstraints (id["grasp_w"], lockbox)
graph.setLevelSetConstraints  (id["keep_grasp_ls"], [], lockbox)
graph.setLockedDofConstraints (id["graph"], locklhand)

p.setInitialConfig (q_init)
p.addGoalConfig (q_goal)

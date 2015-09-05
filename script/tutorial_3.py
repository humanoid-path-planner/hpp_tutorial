from math import sqrt
from hpp.corbaserver.manipulation.dlr_miiwa import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer

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

robot = Robot ('dlr-box', 'dlr')
ps = ProblemSolver (robot)

fk = ViewerFactory (ps)

fk.loadObjectModel (Box, 'box')
fk.loadEnvironmentModel (Environment, "kitchen_area")

robot.setJointBounds ("dlr/miiwa_joint_x", [-4, -3])
robot.setJointBounds ("dlr/miiwa_joint_y", [-5, -3])
robot.setJointBounds ("box/base_joint_xyz", [-5.1,-2,-5.2,-2.7,0,3.])

robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterations (40)
ps.selectPathProjector ('Progressive', 0.2)

# Create lists of joint names - useful to define passive joints. {{{2
jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['dlr'] = list ()
for n in jointNames['all']:
  if n.startswith ("dlr"):
    jointNames['dlr'].append (n)

robotPassiveDof = 'robotPassiveDof'
ps.addPassiveDofs (robotPassiveDof, jointNames['dlr'])
# 2}}}

q_init = robot.getCurrentConfig ()
q_init [0:2] = [-3.4, -4.2]
rank = robot.rankInConfiguration ['dlr/schunk_wsg50_joint_left_jaw']
q_init [rank:rank+2] = [0.05, 0.05]
rank = robot.rankInConfiguration ['box/base_joint_SO3']
c = sqrt (2) / 2
q_init [rank:rank+4] = [c, 0, c, 0]
q_goal = q_init [::]

rank = robot.rankInConfiguration ['box/base_joint_xyz']
q_init [rank:rank+3] = [-2.5, -4, 0.8]
q_goal [rank:rank+3] = [-4.8, -5, 0.9]

del c
# Create constraints to open the gripper
lockHand = ['open_l_jaw', 'open_r_jaw']
ps.createLockedJoint ('open_l_jaw', 'dlr/schunk_wsg50_joint_left_jaw', [0.05,])
ps.createLockedJoint ('open_r_jaw', 'dlr/schunk_wsg50_joint_right_jaw', [0.05,])

# Create placement constraint for the box
ps.client.manipulation.problem.createPlacementConstraint \
('box_placement_table', 'box/base_joint_SO3', 'box/box_surface',
 'kitchen_area/pancake_table_table_top')
ps.client.manipulation.problem.createPlacementConstraint \
('box_placement_counter', 'box/base_joint_SO3', 'box/box_surface',
'kitchen_area/white_counter_top_sink')

# Create the manipulation graph
graph = ConstraintGraph (robot, 'graph')
# create grasp between robot gripper and box "handle". 2 constraints are
# created with name "grasp_box1" and "grasp_box1/complement"
graph.createGrasp ('grasp_box1', 'dlr/schunk_wsg50', 'box/handle',
                   robotPassiveDof)
# create grasp between robot gripper and box "handle2"
graph.createGrasp ('grasp_box2', 'dlr/schunk_wsg50', 'box/handle2',
                   robotPassiveDof)
# create corresponding pre-grasps
graph.createPreGrasp ('pre_grasp_box1', 'dlr/schunk_wsg50', 'box/handle')
graph.createPreGrasp ('pre_grasp_box2', 'dlr/schunk_wsg50', 'box/handle2')


lockBox = ps.lockFreeFlyerJoint ('box/base_joint', 'box_lock')

graph.createNode (['hold1', 'hold2', 'free_table', 'free_counter'])
graph.createEdge ('free_table', 'free_table', 'move_free_table', 1)
graph.createEdge ('free_counter', 'free_counter', 'move_free_counter', 1)
graph.createEdge ('hold1', 'hold1', 'keep_grasp1', 1)
graph.createEdge ('hold2', 'hold2', 'keep_grasp2', 1)
graph.createWaypointEdge ('free_table', 'hold1', 'approach_box1_table', 1, 10)
graph.createWaypointEdge ('free_table', 'hold2', 'approach_box2_table', 1, 10)
graph.createWaypointEdge ('free_counter', 'hold1', 'approach_box1_counter',
                          1, 10)
graph.createWaypointEdge ('free_counter', 'hold2', 'approach_box2_counter',
                          1, 10)

## Set constraints relative to graph element
# Gripper should be open all the time
graph.setConstraints (graph = True, lockDof = lockHand)

# When not grasped, box should lie on the table
graph.setConstraints (node = 'free_table',
                      numConstraints = ['box_placement_table'])
graph.setConstraints (node = 'free_counter',
                      numConstraints = ['box_placement_counter'])
# transfer path: the box should be fixed wrt the gripper
graph.setConstraints (node = 'hold1', grasps = ['grasp_box1',])
graph.setConstraints (node = 'hold2', grasps = ['grasp_box2',])
# transit path: the box should not move
graph.setConstraints (edge = 'move_free_table', lockDof = lockBox)
graph.setConstraints (edge = 'move_free_counter', lockDof = lockBox)
# pre-grasps
graph.setConstraints (edge = 'approach_box1_table_e0', lockDof = lockBox)
graph.setConstraints (edge = 'approach_box1_table_e1', lockDof = lockBox)
graph.setConstraints (node = 'approach_box1_table_n0',
                      pregrasps = ['pre_grasp_box1',])
graph.setConstraints (edge = 'approach_box1_counter_e0', lockDof = lockBox)
graph.setConstraints (edge = 'approach_box1_counter_e1', lockDof = lockBox)
graph.setConstraints (node = 'approach_box1_counter_n0',
                      pregrasps = ['pre_grasp_box1',])
graph.setConstraints (edge = 'approach_box2_table_e0', lockDof = lockBox)
graph.setConstraints (edge = 'approach_box2_table_e1', lockDof = lockBox)
graph.setConstraints (node = 'approach_box2_table_n0',
                      pregrasps = ['pre_grasp_box2',])
graph.setConstraints (edge = 'approach_box2_counter_e0', lockDof = lockBox)
graph.setConstraints (edge = 'approach_box2_counter_e1', lockDof = lockBox)
graph.setConstraints (node = 'approach_box2_counter_n0',
                      pregrasps = ['pre_grasp_box2',])

graph.setConstraints (edge = 'keep_grasp1', grasps = ['grasp_box1',])
graph.setConstraints (edge = 'keep_grasp2', grasps = ['grasp_box2',])

res = ps.client.manipulation.problem.applyConstraints \
      (graph.nodes['free_table'], q_init)
if not res[0]:
  raise Exception ('Init configuration could not be projected.')

q_init_proj = res [1]

res = ps.client.manipulation.problem.applyConstraints \
      (graph.nodes['free_counter'], q_goal)
if not res[0]:
  raise Exception ('Goal configuration could not be projected.')

q_goal_proj = res [1]

ps.setInitialConfig (q_init_proj)
ps.addGoalConfig (q_goal_proj)

r = fk.createRealClient ()
pp = PathPlayer (robot.client.basic, r)

from hpp.corbaserver.pr2 import Robot
from hpp.corbaserver import ProblemSolver
from hpp.gepetto import Viewer
from hpp.gepetto import PathPlayer

# define colors for the roadmap
white=[1.0,1.0,1.0,1.0]
green=[0.23,0.75,0.2,0.5]
brown=[0.85,0.75,0.15,0.5]
blue = [0.0, 0.0, 0.8, 1.0]
grey = [0.7,0.7,0.7,1.0]
red = [0.8,0.0,0.0,1.0]

robot = Robot ('pr2')
robot.setJointBounds ("base_joint_xy", [-4, -3, -5, -3])
ps = ProblemSolver (robot)
v = Viewer (ps)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['torso_lift_joint']
q_init [rank] = 0.2
v (q_init)

q_goal [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['l_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['l_elbow_flex_joint']
q_goal [rank] = -0.5
rank = robot.rankInConfiguration ['r_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['r_elbow_flex_joint']
q_goal [rank] = -0.5
#r (q_goal)

v.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

#ps.selectPathPlanner("rrtConnect")

ps.solve ()

# display roadmap for the base of the robot (no specified joint)
v.displayRoadmap("rmB",white,0.02,1,green)
# display the path found in the roadmap : 
v.displayPathMap("rmPath",0,red,0.03,1,red)
# hide previous roadmap
v.client.gui.removeFromGroup("rmB",v.sceneName)
v.client.gui.removeFromGroup("rmPath",v.sceneName)


#display roadmap for the tools :
v.displayRoadmap("rmR",blue,0.02,1,green,'r_gripper_tool_joint')
v.displayRoadmap("rmL",red,0.02,1,grey,'l_gripper_tool_joint')


# alternative method : replace ps.solve() and v.displayRoadmap() with :
# v.solveAndDisplay("rmR",2,blue,0.02,1,green,'r_gripper_tool_joint')
# v.displayRoadmap("rmL",red,0.02,1,grey,'l_gripper_tool_joint')
################################################################

pp = PathPlayer (robot.client, v)
#display path
pp (0)
#display path with post-optimisation
pp (1)


# hide roadmap in the scene
v.client.gui.removeFromGroup("rmL",v.sceneName)
v.client.gui.removeFromGroup("rmR",v.sceneName)


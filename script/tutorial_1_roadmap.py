from hpp.corbaserver.pr2 import Robot
from hpp.corbaserver import ProblemSolver
from hpp.gepetto import Viewer
from hpp.gepetto import PathPlayer

# define colors for the roadmap (normalized RGBA)
white=[1.0,1.0,1.0,1.0]
lightGreen=[0.23,0.75,0.2,0.5]
green=[0.23,0.75,0.2,1]
yellow=[0.85,0.75,0.15,0.9]
blue = [0.0, 0.0, 0.8, 1.0]
lightBlue = [0.0, 0.0, 0.8, 0.7]
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
#v (q_goal)

v.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.addPathOptimizer ("RandomShortcut")

ps.solve ()

# display roadmap for the base of the robot (no specified joint)
# displayRoadmap(name,sizeNode)
v.displayRoadmap("rmB",0.02)
# display the path found in the roadmap :
# displayPathMap(name,pathID,sizeNode) or : 
# displayPathMap(name,pathID,sizeNode,sizeAxis,colorNode,colorEdge,JointName)
v.displayPathMap("rmPath",0,0.03)
# hide previous roadmap
v.client.gui.removeFromGroup("rmB",v.sceneName)
v.client.gui.removeFromGroup("rmPath",v.sceneName)


# display roadmap for the tools, the full prototype is :
# displayRoadmap(name,sizeNode,sizeAxis,colorNode,colorEdge,JointName)
# The joint defined by "jointName" (see robot.getAllJointNames()) is used to compute the position of the node of the roadmap 
v.displayRoadmap("rmR",0.02,1,blue,lightBlue,'r_gripper_tool_joint')
v.displayRoadmap("rmL",0.02,1,green,lightGreen,'l_gripper_tool_joint')


# alternative method : replace ps.solve() and v.displayRoadmap() with :
# v.solveAndDisplay("rmL",2,0.02,1,green,lightGreen,'l_gripper_tool_joint')
# v.displayRoadmap( "rmR"  ,0.02,1,blue ,lightBlue ,'r_gripper_tool_joint')
# solveAndDisplay show the construction of the roadmap, the first int set the refresh rate of the display (here, every 2 iteration of the algorithm)
################################################################

pp = PathPlayer (robot.client, v)
#display path
# display the curve of the path (default color = yellow)
pp.displayPath(0,jointName='r_gripper_tool_joint')
pp (0)
#display path after optimisation
pp.displayPath(1,yellow,'r_gripper_tool_joint')
pp (1)


# hide roadmap in the scene
v.client.gui.removeFromGroup("rmL",v.sceneName)
v.client.gui.removeFromGroup("rmR",v.sceneName)


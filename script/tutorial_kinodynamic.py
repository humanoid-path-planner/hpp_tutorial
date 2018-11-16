from hpp.corbaserver.robot import Robot
import omniORB.any


# Take a box with a freeflyer base as robot
class RobotBox (Robot):
  rootJointType = 'freeflyer'
  packageName = "hpp_tutorial"
  meshPackageName = "hpp_tutorial"
  urdfName = 'box'
  urdfSuffix = ""
  srdfSuffix = ""
  def __init__ (self, robotName, load = True):
        Robot.__init__ (self, robotName, self.rootJointType, load)
        self.tf_root = "base_footprint"

robot = RobotBox("box")

robot.setJointBounds ("root_joint", [-7, 6.5, -7, 7,0.4,0.4])

# Kinodynamic methods need at least 6 extraConfigs, to store the velocity (first 3) and acceleration (last 3) of the root
robot.client.robot.setDimensionExtraConfigSpace(6)
# set the bounds for velocity and acceleration :
aMax=1.
vMax=2.
robot.client.robot.setExtraConfigSpaceBounds([-vMax,vMax,-vMax,vMax,-vMax,vMax,-aMax,aMax,-aMax,aMax,-aMax,aMax])

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
# define the velocity and acceleration bounds used by the steering method. This bounds will be stastified along the whole trajectory.
ps.client.problem.setParameter("Kinodynamic/velocityBound",omniORB.any.to_any(vMax))
ps.client.problem.setParameter("Kinodynamic/accelerationBound",omniORB.any.to_any(aMax))
ps.client.problem.setParameter("PathOptimization/RandomShortcut/NumberOfLoops",omniORB.any.to_any(100))


# The following line constraint the random sampling method to fix all the extraDOF at 0 during sampling. Comment it if you want to sample states with non-null velocity and acceleration.
ps.client.problem.setParameter("ConfigurationShooter/sampleExtraDOF",omniORB.any.to_any(False))

from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]

q_init [0:3] = [6.5,-4,0.4] #root position
q_init[3:7] = [0,0,0,1] #root rotation
#set initial velocity (along x,y,z axis) : 
q_init[-6:-3]=[0,0,0]


#q_goal[0:3] = [6.5,-1,0.4] # straight line
q_goal [0:3] = [3,-4,0.4] # easy goal position
#q_goal[0:3]=[-4.5,-4.8,0.4]# harder goal position
#set goal velocity (along x,y,z axis) : 
q_goal[-6:-3]=[0,0,0]

vf.loadObstacleModel ("iai_maps", "room", "room")
# with displayArrow parameter the viewer will display velocity and acceleration of the center of the robot with 3D arrow. The length scale with the amplitude with a length of 1 for the maximal amplitude
v = vf.createViewer(displayArrows = True)
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.addPathOptimizer ("RandomShortcut")
#select kinodynamic methods : 
ps.selectSteeringMethod("Kinodynamic")
ps.selectDistance("Kinodynamic")
# the Kinodynamic steering method require a planner that build directionnal roadmap (with oriented edges) as the trajectories cannot always be reversed. 
ps.selectPathPlanner("BiRRTPlanner")


print (ps.solve ())

# display the computed roadmap : 
v.displayRoadmap("rm")

#Alternatively, use the following line instead of ps.solve() to display the roadmap as it's computed (note that it slow down a lot the computation)
#v.solveAndDisplay('rm',1)

# Highlight the solution path used in the roadmap
v.displayPathMap('pm',0)

# remove the roadmap for the scene : 
#v.client.gui.removeFromGroup("rm",v.sceneName)
#v.client.gui.removeFromGroup("pm",v.sceneName)


# Connect to a viewer server and play solution paths
from hpp.gepetto import PathPlayer
pp = PathPlayer (v)
#play path before optimization
#pp (0)

# Display the optimized path, with a color variation depending on the velocity along the path (green for null velocity, red for maximal velocity)
pp.dt=0.1
pp.displayVelocityPath(1)
# play path after random shortcut
pp.dt=0.001
pp (1)




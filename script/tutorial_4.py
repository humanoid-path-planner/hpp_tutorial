from hpp.corbaserver import loadServerPlugin, wrap_delete as wd
from hpp.corbaserver.manipulation import ConstraintGraph, Constraints,\
    newProblem, ProblemSolver, Robot
from hpp.gepetto.manipulation import ViewerFactory
from hpp.rostools import process_xacro, retrieve_resource

## Load robot from processing of a xacro file
Robot.urdfString = process_xacro\
    ("package://hpp_tutorial/urdf/ur10e.urdf.xacro",
     "transmission_hw_interface:=hardware_interface/PositionJointInterface")
Robot.srdfString = ""
loadServerPlugin ("corbaserver", "manipulation-corba.so")
newProblem()

robot = Robot('robot', 'ur10e', rootJointType="anchor")
ps = ProblemSolver(robot)
vf = ViewerFactory(ps)

## Add a gripper to the robot
robot.client.manipulation.robot.addGripper\
    ('ur10e/wrist_3_link', 'ur10e/gripper', [0,0,.1,0.5,0.5,0.5,-0.5], 0.1)

## Create two handles
robot.client.manipulation.robot.addHandle\
    ('ur10e/base_link', 'handle1', [.8, -.4, .5, 0, 0, 0, 1], .1, 6*[True])
robot.client.manipulation.robot.addHandle\
    ('ur10e/base_link', 'handle2', [.8,  .4, .5, 0, 0, 0, 1], .1, 6*[True])

## Create grasp constraints
robot.client.manipulation.problem.createGrasp\
    ("ur10e/gripper grasps handle1", "ur10e/gripper", "handle1")
robot.client.manipulation.problem.createGrasp\
    ("ur10e/gripper grasps handle2", "ur10e/gripper", "handle2")

## Create a constraint graph with one node for each grasp
cg = ConstraintGraph(robot, "graph")
cg.createNode(["ur10e/gripper grasps handle1", "ur10e/gripper grasps handle2"])

cg.addConstraints(node = "ur10e/gripper grasps handle1", constraints =\
    Constraints(numConstraints = ["ur10e/gripper grasps handle1"]))
cg.addConstraints(node = "ur10e/gripper grasps handle2", constraints =\
    Constraints(numConstraints = ["ur10e/gripper grasps handle2"]))
cg.initialize()

## Generate one configuration satisfying each constraint
q0 = 6*[0.]
res, q1, err = cg.applyNodeConstraints("ur10e/gripper grasps handle1", q0)
# Use q1 to generate q2 to minimize the probability of discontinuities
res, q2, err = cg.applyNodeConstraints("ur10e/gripper grasps handle2", q1)

## Create an EndEffectorTrajectory steering method
cmp = wd(ps.client.basic.problem.getProblem())
crobot = wd(cmp.robot())
cproblem = wd(ps.client.basic.problem.createProblem(crobot))
csm = wd(ps.client.basic.problem.createSteeringMethod("EndEffectorTrajectory",
                                                      cproblem))
cs = wd(ps.client.basic.problem.createConstraintSet(crobot, "sm-constraints"))
cp = wd(ps.client.basic.problem.createConfigProjector(crobot, "cp", 1e-4, 40))
cs.addConstraint(cp)
csm.setConstraints(cs)

# Create a new grasp constraint for the steering method right hand side
# The previously created one has EqualToZero as comparison types.
robot.client.manipulation.problem.createGrasp\
    ("end-effector-tc", "ur10e/gripper", "handle1")
# Set comparison type to Equality
ps.setConstantRightHandSide("end-effector-tc", False)
tc = wd(ps.client.basic.problem.getConstraint("end-effector-tc"))
cp.add(tc, 0)

csm.trajectoryConstraint(tc)
# Get right hand side for q1 and q2
rhs1 = tc.function().value(q1)
rhs2 = tc.function().value(q2)
# Create linear path for end-effector
p = wd(csm.makePiecewiseLinearTrajectory([rhs1, rhs2], 6*[1.]))
# Set this path as the time-varying right hand side of the constraint
csm.trajectory(p, True)

## Call steering method
p1 = wd(csm.call(q1,q2))
ps.client.basic.problem.addPath(p1.asVector())

# Right now, path projectors do not work with EndEffectorTrajectory steering
# method

//
// Copyright (c) 2017 CNRS
// Authors: Florent Lamiraux
//
//
// This file is part of hpp_tutorial
// hpp_tutorial is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp_tutorial is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp_tutorial  If not, see
// <http://www.gnu.org/licenses/>.

/// \page hpp_tutorial_tutorial_4 Tutorial 4 - inverse kinematics
///
/// To run this tutorial, you need to install package \c ur_description.
///
/// To run the tutorial, open a terminal and open 3 tabs by typing
/// \c CTRL+SHIFT+T twice. When the terminal is selected, you can select a tab
/// by typing \c ALT-[1|2|3].
///
/// \section hpp_tutorial_4_starting_hpp_manipulation_server Starting
/// hppcorbaserver
///
/// In the first tab, type
/// \code
/// hppcorbaserver
/// \endcode
/// See package \c hpp-manipulation-corba for details.
///
/// \section hpp_tutorial_4_python Controlling via a python terminal
///
/// In the second tab, type
/// \code
/// cd script
/// python -i tutorial_4.py
/// \endcode
/// Script <code><a href="script/tutorial_4.py">script/tutorial_4.py</a></code>
/// defines a path for an UR10 end-effector
///
/// \section hpp_tutorial_4_starting_gui Starting gepetto-gui
///
/// In the third tab, type
/// \code
/// gepetto-gui
/// \endcode
/// A window opens and is ready to display the scene containing the robot. The
/// robot, environment and object will appear later.
///
/// \section hpp_tutorial_4_python Controlling via a python terminal
///
/// To display the scene, create a client to the viewer in the python terminal.
/// \code
/// >>> v = vf.createViewer ()
/// \endcode
/// The robot and environment should appear in the viewer. If the viewer
/// window is black, select the window and hit space.
///
/// \section hpp_tutorial_4_explanition Explaining the script
///
/// The first part of the script below
/// \li creates the robot from a xacro file,
/// \li creates the problem solver and the viewer factory.
///
/// \code
/// ## Load robot from processing of a xacro file
/// Robot.urdfString = process_xacro\
///     ("package://hpp_tutorial/urdf/ur10e.urdf.xacro",
///      "transmission_hw_interface:=hardware_interface/PositionJointInterface")
/// # Deactivate collision checking between consecutive links
/// Robot.srdfString = """
/// <robot name="ur10e">
///   <disable_collisions link1="shoulder_link"
///      link2="upper_arm_link" reason=""/>
///   <disable_collisions link1="upper_arm_link"
/// 		      link2="forearm_link" reason=""/>
///
///   <disable_collisions link1="wrist_1_link"
/// 		      link2="wrist_2_link" reason=""/>
///   <disable_collisions link1="wrist_2_link"
/// 		      link2="wrist_3_link" reason=""/>
///   <disable_collisions link1="shoulder_link"
/// 		      link2="forearm_link" reason=""/>
///   <disable_collisions link1="wrist_1_link"
/// 		      link2="wrist_3_link" reason=""/>
///   <disable_collisions link1="base_link_inertia"
/// 		      link2="shoulder_link" reason=""/>
///   <disable_collisions link1="forearm_link"
/// 		      link2="wrist_1_link" reason=""/>
/// </robot>
/// """
/// loadServerPlugin ("corbaserver", "manipulation-corba.so")
/// newProblem()
///
/// robot = Robot('robot', 'ur10e', rootJointType="anchor")
/// ps = ProblemSolver(robot)
/// vf = ViewerFactory(ps)
/// \endcode
///
/// The following lines
/// \li create a gripper attached to the end-effector of the robot,
/// \li create two handles attached to the base of the robot.
///
/// These grippers and handles will ease the creation of pose constraints for
/// the end-effector.
///
/// \code
/// ## Add a gripper to the robot
/// robot.client.manipulation.robot.addGripper\
///     ('ur10e/wrist_3_link', 'ur10e/gripper', [0,0,.1,0.5,0.5,0.5,-0.5], 0.1)
///
/// ## Create two handles
/// robot.client.manipulation.robot.addHandle\
///     ('ur10e/base_link', 'handle1', [.8, -.4, .5, 0, 0, 0, 1], .1, 6*[True])
/// robot.client.manipulation.robot.addHandle\
///     ('ur10e/base_link', 'handle2', [.8,  .4, .5, 0, 0, 0, 1], .1, 6*[True])
/// \endcode
///
/// Then two grasp constraints are created to defined initial and goal pose
/// of the gripper
///
/// \code
/// ## Create grasp constraints
/// robot.client.manipulation.problem.createGrasp\
///     ("ur10e/gripper grasps handle1", "ur10e/gripper", "handle1")
/// robot.client.manipulation.problem.createGrasp\
///     ("ur10e/gripper grasps handle2", "ur10e/gripper", "handle2")
/// \endcode
///
/// Configurations satisfying the pose constraints are computed by creating
/// a constraint graph with two nodes and no edge.
///
/// \code
/// ## Create a constraint graph with one node for each grasp
/// cg = ConstraintGraph(robot, "graph")
/// cg.createNode(["ur10e/gripper grasps handle1", "ur10e/gripper grasps
/// handle2"])
/// cg.addConstraints(node = "ur10e/gripper grasps handle1", constraints = \
///     Constraints(numConstraints = ["ur10e/gripper grasps handle1"]))
/// cg.addConstraints(node = "ur10e/gripper grasps handle2", constraints = \
///     Constraints(numConstraints = ["ur10e/gripper grasps handle2"]))
/// cg.initialize()
///\endcode
///
/// We compute the initial and goal configurations by numerical inverse
/// kinematics.
///
/// \code
/// # Generate one configuration satisfying each constraint
/// found = False
/// while not found:
///     q0 = robot.shootRandomConfig()
///     res, q1, err = cg.applyNodeConstraints("ur10e/gripper grasps handle1", q0)
///     if not res: continue
///     res, msg = robot.isConfigValid(q1)
///     if not res: continue
///     res, q2, err = cg.applyNodeConstraints("ur10e/gripper grasps handle2", q1)
///     if not res: continue
///     res, msg = robot.isConfigValid(q2)
///     if not res: continue
///     found = True
///
/// We create several CORBA objects:
/// \li the current manipulation planning problem \c cmp,
/// \li the robot stored in this problem \c crobot,
/// \li a steering method \c csm of type
/// hpp::manipulation::steeringMethod::EndEffectorTrajectory
///
/// \code
/// ## Create an EndEffectorTrajectory steering method
/// cmp = wd(ps.client.basic.problem.getProblem())
/// crobot = wd(cmp.robot())
/// cproblem = wd(ps.client.basic.problem.createProblem(crobot))
/// csm =
/// wd(ps.client.basic.problem.createSteeringMethod("EndEffectorTrajectory",
/// cproblem)) \endcode
///
/// Then we create a \link hpp::core::ConstraintSet ContraintSet \endlink
/// containing an empty \link hpp::core::ConfigProjector ConfigProjector
/// \endlink that we pass to the problem. We set  \link
/// hpp::manipulation::steeringMethod::EndEffectorTrajectory \c csm \endlink as
/// the steering method of the problem. Note that the last line passes the \link
/// hpp::core::ConstraintSet ContraintSet \endlink of the problem to the
/// steering method. The order is important here since at construction the
/// problem is given an empty \link hpp::core::ConstraintSet ContraintSet
/// \endlink and setting the steering method of
/// the problem passes the \link hpp::core::ConstraintSet ContraintSet \endlink
/// of the problem to the steering method.
///
/// \code
/// cs = wd(ps.client.basic.problem.createConstraintSet(crobot,
/// "sm-constraints")) cp =
/// wd(ps.client.basic.problem.createConfigProjector(crobot, "cp", 1e-4, 40))
/// cs.addConstraint(cp)
/// cproblem.setConstraints(cs)
/// cproblem.setSteeringMethod(csm)
/// \endcode
///
/// We need to create a time varying constraint for the end-effector. For that,
/// we create a new grasp between the gripper and the first handle. Note that
/// we cannot use the previously created identical grasp, since the comparison
/// type of this one should be \c Equality.
///
/// \code
/// # Create a new grasp constraint for the steering method right hand side
/// # The previously created one has EqualToZero as comparison types.
/// robot.client.manipulation.problem.createGrasp         \
///     ("end-effector-tc", "ur10e/gripper", "handle1")
/// # Set comparison type to Equality
/// ps.setConstantRightHandSide("end-effector-tc", False)
/// \endcode
///
/// We insert this constraint into the \link hpp::core::ConfigProjector
/// ConfigProjector \endlink of the \link hpp::core::Problem problem \endlink
/// (and thus of the steering method)
///
/// \code
/// tc = wd(ps.client.basic.problem.getConstraint("end-effector-tc"))
/// cp.add(tc, 0)
/// \endcode
///
/// We pass this constraint to the steering method as the trajectory constraint.
/// Note that from a mathematical point of view, the trajectory constraint is
/// a mapping from the robot configuration space \f$\mathcal{C}\f$ to
/// \f$SE(3)\f$ defined by
/// \f{equation}
/// tc(\mathbf{q}) = g^{-1}(\mathbf{q})h_1
/// \f}
/// where \f$g(\mathbf{q})\in SE(3)\f$ is the pose of the gripper in
/// configuration \f$\mathbf{q}\f$ and \f$h_1\in SE(3)\f$ is the pose of
/// \c handle1.
///
/// \code
/// csm.trajectoryConstraint(tc)
/// \endcode
///
/// We now need to build the right hand side of the constraint as a linear
/// interpolation between the initial and final values. For that we evaluate
/// the fonction \f$tc\f$ of the constraint at the initial and goal
/// configurations,
/// we create a path in SE(3) linking these two values and we give this path
/// to the steering method to define the time-varying right hand side.
///
/// From a mathematical point of view, \f$\mathbf{p}\f$ is a mapping from an
/// interval \f$[0,T]\f$ to \f$SE(3)\f$ such that
/// \f{eqnarray}
/// \mathbf{p}(0) &=& tc(\mathbf{q}_1) \\
/// \mathbf{p}(T) &=& tc(\mathbf{q}_2) \\
/// \f}
/// The constraint applied along the path computed by the steering method is
/// thus:
/// \f{equation}
/// \forall t\in[0,T],\ \ \mathbf{tc}(\mathbf{q}(t)) = \mathbf{p}(t)
/// \f}
///
/// \code
/// # Get right hand side for q1 and q2
/// rhs1 = tc.function().value(q1)
/// rhs2 = tc.function().value(q2)
/// # Create linear path for end-effector
/// p = wd(csm.makePiecewiseLinearTrajectory([rhs1, rhs2], 6*[1.]))
/// # Set this path as the time-varying right hand side of the constraint
/// csm.trajectory(p, True)
///\endcode
///
/// We can now call the steering method between the initial and goal
/// configurations and insert this path in the ProblemSolver in order to
/// make it visible in \c gepetto-gui.
///
/// \code
/// ## Call steering method
/// p1 = wd(csm.call(q1,q2))
/// if p1:
///    ps.client.basic.problem.addPath(p1.asVector())
///\endcode
///
/// After connecting and refreshing \c gepetto-gui, you should be able to
/// display the path. Notice that the path might be discontinuous.
///
/// To get a continuous path for sure, we need to use the \link
/// hpp::manipulation::pathPlanner::EndEffectorTrajectory EndEffectorTrajectory
/// path planner \endlink.
///
/// \code
/// ## Using EndEffectorTrajectory path planner
/// cdistance = wd(cproblem.getDistance())
/// croadmap = wd(ps.client.basic.problem.createRoadmap(cdistance, crobot))
/// cplanner = wd(ps.client.basic.problem.createPathPlanner(
///     "EndEffectorTrajectory", cproblem, croadmap))
/// cplanner.setNRandomConfig(0)
/// cplanner.maxIterations(1)
/// cplanner.setNDiscreteSteps(20)
///
/// cproblem.setInitConfig(q1)
/// cproblem.addGoalConfig(q2)
///
/// p2 = wd(cplanner.solve())
/// if p2:
///     ps.client.basic.problem.addPath(p2)
///\endcode
///
/// We set the number of random configurations to 0 in order to force the
/// planner to start from \c q1. Otherwise, in case of failure to plan a
/// continuous path starting from \c q1, the planner would generate random
/// initial configurations that satisfy the constraints at the beginning.
/// To be consistent, we set the maximal number of iterations to 1 since new
/// iterations would simply retry to start from \c q1. We set the number of
/// steps at which a configuration is computed for the corresponding pose of the
/// end effector to 20.
///
/// Notice that the path satisfies the end-effector time-varying constraint, but
/// does not necessarily end at \f$\mathbf{q}_2\f$ since the final
/// configuration is completely determined by the initial one.

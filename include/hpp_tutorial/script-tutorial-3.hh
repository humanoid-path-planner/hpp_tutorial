//
// Copyright (c) 2015 CNRS
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

/// \page hpp_tutorial_script_3 Explanation about script/tutorial_3.py
///
/// \code{.py}
/// from math import sqrt, pi
/// from hpp.corbaserver.manipulation.hpp_dlr_ipa import Robot
/// from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph
/// from hpp.gepetto.manipulation import ViewerFactory
/// from hpp.gepetto import PathPlayer
/// \endcode
/// Import required classes. ViewerFactory allows one to create a
/// dummy visualiser. Useful if there is no need to visualise the
/// problem until later (done by calling r.createRealClient ()).
/// Alternatively, import 'Viewer' from hpp.gepetto to visualise from the beginning.
///
/// \code {.py}
/// class Door (object):
///    rootJointType = 'anchor'
///    packageName = 'hpp-dlr-ipa'
///    meshPackageName = 'hpp-dlr-ipa'
///    urdfName = 'door'
///    urdfSuffix = ""
///    srdfSuffix = ""
/// \endcode
///
/// Create door class to load urdf model of the required object. This
/// class contains information about the installed ROS-package were urdf
/// models and collada/stl files can be found. Note that those packages need to
/// be installed.
/// \code {.py}
/// robot = Robot ('ipa-box', 'ipa')
/// robot.client.manipulation.robot.setRootJointPosition('ipa', [0,0,0,1,0,0,0])
///\endcode
/// Create manipulation robot and set its initial position. The kinematic chain is initialized with the
/// fraunhofer ipa robot. The first parameter is the name of the composite robot
/// (robotic arm + platform). The second argument is the name of the fraunhofer
/// robot in the composite robot (only the robotic arm).
/// \code {.py}
/// ps = ProblemSolver (robot)
/// ps.addPathOptimizer ('Graph-RandomShortcut')
/// \endcode
/// Create a problem solver and set a path optimiser.
/// \sa manipulation.problem_solver.ProblemSolver python class.
///
/// To view the selection of available optimisers, call:
/// ps.client.basic.problem.getAvailable('PathOptimizer').
/// Optimisers with the prefix 'Graph' may be used in manipulation tasks.
/// To undo path optimiser selection, call \c ps.clearPathOptimizers ()
///
/// \code {.py}
/// r = ViewerFactory (ps)
/// \endcode
/// Create a dummy client to \c hpp-manipulation-server and to \c gepetto-viewer-server
/// \code {.py}
/// r.loadObjectModel (Door, 'door')
/// robot.client.manipulation.robot.setRootJointPosition('door', [0.5,1.6,0.8,1,0,0,0])
/// \endcode
/// Load urdf model of object and add it as a kinematic chain to the composite
/// robot.
/// \code {.py}
/// robot.client.basic.robot.setDimensionExtraConfigSpace (4)
/// robot.client.basic.robot.setExtraConfigSpaceBounds ([0,1, 0,1, 0,1, 0,1])
/// \endcode
/// Create an extra degree of freedom to execute tasks that seemignly have the same
/// initial and goal configuration (which is generally not supported by the path planner).
/// By changing the value of the extra DoF, the successful execution of the task is registered.
/// \code {.py}
/// graph = ConstraintGraph (robot, 'graph')
/// \endcode
/// Create the constraint graph.
/// \code {.py}
/// jointNames = dict ()
/// jointNames['all'] = robot.getJointNames ()
/// jointNames['ipa'] = list ()
/// for n in jointNames['all']:
///   if n.startswith ("ipa"):
///     jointNames['ipa'].append (n)
///     ps.client.manipulation.problem.createLockedJoint (n, n, [0])
/// \endcode
/// Create a dictonary the values of which are lists of joint names.
/// This is in general useful in order to define passive joints (not used in this tutorial).
/// \code {.py}
/// lockRobot = jointNames['all'][:]
/// robotPassiveDof = 'robotPassiveDof'
/// ps.addPassiveDofs (robotPassiveDof, jointNames['ipa'])
/// \endcode
/// Store a list of joint names as possible passive joints for latter
/// constraints.
/// \code {.py}
/// q_init = [0, -pi/2, 0, -pi/2, 0, 0, 0, 0, 0, 0]
/// r(q_init)
/// \endcode
/// Give initial configuration (note extraDoFs that extend config size).
///
/// \code {.py}
/// graph.createGrasp ('first', 'ipa/screwdriver', 'door/door_lower_right_corner')
/// graph.createPreGrasp ('firstPre', 'ipa/screwdriver', 'door/door_lower_right_corner')
/// ps.client.manipulation.problem.createLockedExtraDof ('extra1', 0, [0])
///
/// graph.createGrasp ('second', 'ipa/screwdriver', 'door/window_lower_left_corner')
/// graph.createPreGrasp ('secondPre', 'ipa/screwdriver', 'door/window_lower_left_corner')
/// ps.client.manipulation.problem.createLockedExtraDof ('extra2', 1, [0])
///
/// graph.createGrasp ('third', 'ipa/screwdriver', 'door/window_lower_right_corner')
/// graph.createPreGrasp ('thirdPre', 'ipa/screwdriver', 'door/window_lower_right_corner')
/// ps.client.manipulation.problem.createLockedExtraDof ('extra3', 2, [0])
///
/// graph.createGrasp ('fourth', 'ipa/screwdriver', 'door/door_handle')
/// graph.createPreGrasp ('fourthPre', 'ipa/screwdriver', 'door/door_handle')
/// ps.client.manipulation.problem.createLockedExtraDof ('extra4', 3, [0])
/// \endcode
/// Create grasps with name, associate gripper and wanted handle to grasp.
/// Create locked extraDof -> Dof must be locked when not in contact with 'door' in order
/// to detect successful execution of task. Constraints are created by name.
/// \sa hpp::model::Gripper and hpp::manipulation::Handle.
///
/// Also create pregrasp constraints. A pregrasp constraint is a position of the
/// gripper in front of the handle in order to approach the object before
/// grasping it. This is an optimisation that eases to find collision-free
/// grasping motions.
///
/// \code {.py}
/// graph.createNode (['door1','door2','door3','door4','free1',])
/// \endcode
/// Create nodes of the constraint graph. Nodes are ordered. When searching
/// to which node a configuration belongs, the search is performed in the order
/// provided here. Thus if a configuration belongs to several nodes, the first
/// one will be found. The order is thus essential: first comes the most restrictive node,
/// followed by those less so.
/// \code {.py}
/// graph.createWaypointEdge ('free1', 'door1', "grasp1", nb=1, weight=1)
/// graph.createWaypointEdge ('free1', 'door2', "grasp2", nb=1, weight=1)
/// graph.createWaypointEdge ('free1', 'door3', "grasp3", nb=1, weight=1)
/// graph.createWaypointEdge ('free1', 'door4', "grasp4", nb=1, weight=1)
/// \endcode
/// Create waypoint edges. The two last parameters correspond to the number of
/// intermediate waypoints and to the weight. Waypoint edges make reaching the goal state easier.
///
/// A waypoint edge is an edge of the constraint graph that contains a chain
/// of (n+1) edges and (n) nodes. In this case, it is used to define an
/// approaching position of the gripper in order to grasp the object.
/// Type help(graph.createWaypointEdge), or see
/// hpp::manipulation::graph::WaypointEdge for details about waypoint edges.
///
/// \code {.py}
/// graph.createEdge ('free1', 'free1', 'move_free1', 0)
/// graph.createLevelSetEdge ('door1', 'door1', 'keep_grasp1', 5)
/// graph.createLevelSetEdge ('door2', 'door2', 'keep_grasp2', 5)
/// graph.createLevelSetEdge ('door3', 'door3', 'keep_grasp3', 5)
/// graph.createLevelSetEdge ('door4', 'door4', 'keep_grasp4', 5)
/// \endcode
/// Create edges of the manipulation graph (gripper in movement when free and in contact).
/// The last parameter is the weight of the edge when randomly choosing an out-edge from a node.
///
/// \code {.py}
/// graph.setConstraints (edge='move_free1', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])
///
/// graph.setConstraints (node='door1', grasps = ['first',])
/// graph.setConstraints (edge='keep_grasp1', lockDof=['extra2', 'extra3', 'extra4']+lockRobot)
/// graph.setLevelSetConstraints (edge='keep_grasp1', lockDof=['extra1',])
///
/// graph.setConstraints (node="grasp1_n0", pregrasps = ['firstPre',])
/// graph.setConstraints (edge='grasp1_e0',  lockDof=['extra1','extra2', 'extra3', 'extra4'])
/// graph.setConstraints (edge='grasp1_e1',  lockDof=['extra1','extra2', 'extra3', 'extra4'])
/// graph.client.graph.setShort (graph.edges["grasp1_e1"], True)
///
/// graph.setConstraints (node='door2', grasps = ['second',])
/// graph.setConstraints (edge='keep_grasp2', lockDof=['extra1', 'extra3', 'extra4']+lockRobot)
/// graph.setLevelSetConstraints (edge='keep_grasp2', lockDof=['extra2',])
///
/// graph.setConstraints (node="grasp2_n0", pregrasps = ['secondPre',])
/// graph.setConstraints (edge='grasp2_e0', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])
/// graph.setConstraints (edge='grasp2_e1', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])
/// graph.client.graph.setShort (graph.edges["grasp2_e1"], True)
///
/// graph.setConstraints (node='door3', grasps = ['third',])
/// graph.setConstraints (edge='keep_grasp3', lockDof=['extra1', 'extra2', 'extra4']+lockRobot)
/// graph.setLevelSetConstraints (edge='keep_grasp3', lockDof=['extra3',])
///
/// graph.setConstraints (node="grasp3_n0", pregrasps = ['thirdPre',])
/// graph.setConstraints (edge='grasp3_e0', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])
/// graph.setConstraints (edge='grasp3_e1', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])
/// graph.client.graph.setShort (graph.edges["grasp3_e1"], True)
///
/// graph.setConstraints (node='door4', grasps = ['fourth',])
/// graph.setConstraints (edge='keep_grasp4', lockDof=['extra1', 'extra2', 'extra3']+lockRobot)
/// graph.setLevelSetConstraints (edge='keep_grasp4', lockDof=['extra4',])
///
/// graph.setConstraints (node="grasp4_n0", pregrasps = ['fourthPre',])
/// graph.setConstraints (edge='grasp4_e0', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])
/// graph.setConstraints (edge='grasp4_e1', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])
/// graph.client.graph.setShort (graph.edges["grasp4_e1"], True)
/// \endcode
/// Set constraints to be applied to all nodes and edges.
/// When moving without touching the desired point (moving to or from the 'door1' node),
/// the extra DoFs have to be locked (change of value impeded).
/// Additionally, the extra DoFs that should not go through any change of value
/// must be locked when in contact as well. Thus, only one extra DoF registers
/// a change when the desired point is reached.
/// \code {.py}
/// q_goal = q_init [::]
/// q_goal [6] = 1
/// ps.setInitialConfig (q_init)
/// ps.addGoalConfig (q_goal)
/// \endcode
/// Initial and Goal configurations must differ from each other.
/// \code {.py}
/// ps.solve ()
/// \endcode
/// Solve for first contact. The solve function adds computed path into a
/// path vector. If path optimisers are used, multiple paths are added
/// into the vector (unoptimised, optimised with 1st optimiser,
/// optimised with 2nd optimiser etc.)
/// \code {.py}
/// q_goal = q_init[::]
/// q_goal [7] = 1
/// ps.resetGoalConfigs()
/// ps.addGoalConfig (q_goal)
/// \endcode
/// to solve for second contact, reset goal configurations and
/// set a new one.
/// \code {.py}
/// ps.solve()
/// \endcode
/// Solve for 2nd contact.
/// \code {.py}
/// q_goal = q_init[::]
/// q_goal [8] = 1
/// ps.resetGoalConfigs()
/// ps.addGoalConfig (q_goal)
/// ps.solve()
/// \endcode
/// Solve for 3rd contact.
/// \code {.py}
/// q_goal = q_init[::]
/// q_goal [9] = 1
/// ps.resetGoalConfigs()
/// ps.addGoalConfig (q_goal)
/// ps.solve()
/// \endcode
/// Solve for last contact.
/// \code {.py}
/// v = r.createRealClient ()
/// \endcode
/// Create real viewer from factory to visualise computed paths.
/// \code {.py}
/// pp = PathPlayer (robot.client.basic, v)
/// pp (0) # visualise first task without optimisation
/// pp (1) #   		-..-	with optimisation
/// pp (3) # 2nd optimised
/// pp (5) # 3rd optimised
/// pp (7) # 4th optimised
/// \endcode
/// Create a path player. To play paths, use the index of
/// computed path in path vector.
///
/// Use in separate window to interrupt \c ps.solve() :
/// \code {.py}
/// from hpp.corbaserver import Client
/// client = Client()
/// client.problem.interruptPathPlanning()
/// \endcode

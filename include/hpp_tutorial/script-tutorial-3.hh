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
/// from math import sqrt
/// from hpp.corbaserver.manipulation.dlr_miiwa import Robot
/// from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph
/// from hpp.gepetto.manipulation import Viewer, ViewerFactory
/// from hpp.gepetto import PathPlayer
/// \endcode
/// Import various classes.
/// class Box (object):
///   rootJointType = 'freeflyer'
///   packageName = 'hpp_tutorial'
///   meshPackageName = 'hpp_tutorial'
///   urdfName = 'box'
///   urdfSuffix = ""
///   srdfSuffix = ""
///
/// class Environment (object):
///   packageName = 'iai_maps'
///   meshPackageName = 'iai_maps'
///   urdfName = 'kitchen_area'
///   urdfSuffix = ""
///   srdfSuffix = ""
/// \endcode
/// Create classes to load urdf models of objects and environments. These
/// classes contain information about the installed ROS-packages were urdf
/// models and collada files can be found. Note that those packages need to
/// be installed.
/// \code {.py}
/// robot = Robot ('dlr-box', 'dlr')
///\endcode
/// Create manipulation robot. The kinematic chain is initialized with the
/// dlr_miiwa robot. The first parameter is the name of the composite robot.
/// The second argument is the name of the dlr_miiwa robot in the composite
/// robot.
/// \code {.py}
/// ps = ProblemSolver (robot)
/// \endcode
/// Create a problem solver.
/// \code {.py}
///r = Viewer (ps)
/// \endcode
/// Create a client to \c hpp-manipulation-server and to \c gepetto-viewer-server
/// \code {.py}
/// r.loadObjectModel (Box, 'box')
/// \endcode
/// Load urdf model of object and add it as a kinematic chain to the composite
/// robot.
/// \code {.py}
/// r.loadEnvironmentModel (Environment, "kitchen_area")
/// \endcode
/// Load model of the environment.
/// \core {.py}
/// robot.setJointBounds ("dlr/miiwa_joint_x", [-4, -3])
/// robot.setJointBounds ("dlr/miiwa_joint_y", [-5, -3])
/// robot.setJointBounds ("box/base_joint_xyz", [-5.1,-2,-5.2,-2.7,0,3.])
/// \endcode
/// Define bounds on translation degrees of freedom of mobile manipulator and
/// of box. Note that the name of the joints is prefixed by the name of the
/// robot in the composite robot.
/// \code {.py}
/// robot.client.basic.problem.setErrorThreshold (1e-3)
/// robot.client.basic.problem.setMaxIterations (40)
/// \endcode
/// Set error threshold and maximal number of iterations for non-linear
/// constraint projection.
/// \code {.py}
/// ps.selectPathProjector ('Progressive', 0.2)
/// \endcode
/// Select strategy to apply non-linear constraints along a path.
/// \code {.py}
/// # Create lists of joint names - useful to define passive joints.
/// jointNames = dict ()
/// jointNames['all'] = robot.getJointNames ()
/// jointNames['dlr'] = list ()
/// for n in jointNames['all']:
///   if n.startswith ("dlr"):
///     jointNames['dlr'].append (n)
/// \endcode
/// Create a dictonary the values of which are lists of joints.
/// \code {.py}
/// robotPassiveDof = 'robotPassiveDof'
/// ps.addPassiveDofs (robotPassiveDof, jointNames['dlr'])
/// \endcode
/// Store a list of joint names as possible passive joints for latter
/// constraints.
/// \code {.py}
/// q_init = robot.getCurrentConfig ()
/// q_init [0:2] = [-3.4, -4.2]
/// rank = robot.rankInConfiguration ['dlr/schunk_wsg50_joint_left_jaw']
/// q_init [rank:rank+2] = [0.05, 0.05]
/// rank = robot.rankInConfiguration ['box/base_joint_SO3']
/// q_init [rank:rank+4] = [sqrt (2)/2, 0, sqrt (2)/2, 0]
/// q_goal = q_init [::]
/// rank = robot.rankInConfiguration ['box/base_joint_xyz']
/// q_init [rank:rank+3] = [-2.5, -4, 0.8]
/// q_goal [rank:rank+3] = [-4.8, -5, 0.9]
/// \endcode
/// Define initial and goal configurations.
///
/// # Create constraints to open the gripper
/// lockHand = ['open_l_jaw', 'open_r_jaw']
/// ps.createLockedJoint ('open_l_jaw', 'dlr/schunk_wsg50_joint_left_jaw', [0.05,])
/// ps.createLockedJoint ('open_r_jaw', 'dlr/schunk_wsg50_joint_right_jaw', [0.05,])
/// \endcode
/// Create constraints to open the gripper. These constraints will be used
/// when defining the constraint graph.
/// \code {.py}
/// # Create placement constraint for the box
/// ps.client.manipulation.problem.createPlacementConstraint \
/// ('box_placement_table', 'box/base_joint_SO3', 'box/box_surface',
///  'kitchen_area/pancake_table_table_top')
/// ps.client.manipulation.problem.createPlacementConstraint \
/// ('box_placement_counter', 'box/base_joint_SO3', 'box/box_surface',
/// 'kitchen_area/white_counter_top_sink')
/// \endcode
/// Create placement constraints for the box on the table and on the counter.
/// Contact surfaces are defined in srdf files of robots and objects.
/// \code {.py}
/// # Create the manipulation graph
/// graph = ConstraintGraph (robot, 'graph')
/// \endcode
/// Create the manipulation graph
/// \code {.py}
/// # create grasp between robot gripper and box "handle". 2 constraints are
/// # created with name "grasp_box1" and "grasp_box1/complement"
/// graph.createGrasp ('grasp_box1', 'dlr/schunk_wsg50', 'box/handle',
///                    robotPassiveDof)
/// \endcode
/// Create grasp between robot gripper and box "handle". 2 constraints are
/// created with name
/// \li "grasp_box1", and
/// \li "grasp_box1/complement".
/// \sa hpp::model::Gripper and hpp::manipulation::Handle.
///
/// The first one is stored in a node of the manipulation graph. The second
/// one is applied along paths that are included in the node. In this case,
/// "grasp_box1/complement" is empty since "grasp_box1" is a full transformation
/// constraint between the gripper and the object handle. In other cases like
/// axial handles, the first constraint is of dimension 5 since the handle has
/// free orientation around the z-axis. The complementary constraint is required
/// to keep the object in a fixed position with respect to the gripper along
/// transfer paths.
/// \li \c robotPassiveDof states that the mobile manipulator degrees of freedom
/// will not be used to solve node constraints relative to this grasp.
///
/// \code {.py}
/// # create grasp between robot gripper and box "handle2"
/// graph.createGrasp ('grasp_box2', 'dlr/schunk_wsg50', 'box/handle2',
///                    robotPassiveDof)
/// \endcode
/// Create grasp constraint with the second handle of the box.
/// \code {.py}
/// # create corresponding pre-grasps
/// graph.createPreGrasp ('pre_grasp_box1', 'dlr/schunk_wsg50', 'box/handle')
/// graph.createPreGrasp ('pre_grasp_box2', 'dlr/schunk_wsg50', 'box/handle2')
/// \endcode
/// Create a pregrasp constraint. A pregrasp constraint is a position of the
/// gripper in front of the handle in order to approach the object before
/// grasping it. This is an optimization that eases to find collision-free
/// grasping motions.
/// \code {.py}
/// lockBox = ps.lockFreeFlyerJoint ('box/base_joint', 'box_lock')
/// \endcode
/// Create a constraint that fix the position of the box in the environment.
/// \code {.py}
/// graph.createNode (['hold1', 'hold2', 'free_table', 'free_counter'])
/// \endcode
/// Create nodes of the constraint graph. Nodes are ordered. When searching
/// to which node a configuration belongs, the search is performed in the order
/// provided here. Thus if a configuration belongs to several nodes, the first
/// one will be found.
/// \code {.py}
/// graph.createEdge ('free_table', 'free_table', 'move_free_table', 1)
/// graph.createEdge ('free_counter', 'free_counter', 'move_free_counter', 1)
/// graph.createEdge ('hold1', 'hold1', 'keep_grasp1', 1)
/// graph.createEdge ('hold2', 'hold2', 'keep_grasp2', 1)
/// \endcode
/// Create edges of the manipulation graph. The last parameter is the weight
/// of the edge when randomly choosing an out-edge from a node.
/// \code {.py}
/// graph.createWaypointEdge ('free_table', 'hold1', 'approach_box1_table', 1, 10)
/// graph.createWaypointEdge ('free_table', 'hold2', 'approach_box2_table', 1, 10)
/// graph.createWaypointEdge ('free_counter', 'hold1', 'approach_box1_counter', 1, 10)
/// graph.createWaypointEdge ('free_counter', 'hold2', 'approach_box2_counter', 1, 10)
/// \endcode
/// Create waypoint edges. The two last parameters correspond to the number of
/// intermediate waypoints and to the weight.
///
/// A waypoint edge is an edge of the constraint graph that contains a chain
/// of (n+1) edges and (n) nodes. In this case, it is used to define an
/// approaching position of the gripper in order to grasp the object.
///
/// See hpp::manipulation::graph::WaypointEdge for details about waypoint edges.
/// \code {.py}
/// ## Set constraints relative to graph element
/// # Gripper should be open all the time
/// graph.setConstraints (graph = True, lockDof = lockHand)
/// \endcode
/// Set constraints to be applied to all nodes and edges.
/// \code {.py}
/// # When not grasped, box should lie on the table
/// graph.setConstraints (node = 'free_table',
///                       numConstraints = ['box_placement_table'])
/// graph.setConstraints (node = 'free_counter',
///                       numConstraints = ['box_placement_counter'])
/// \endcode
/// Set constraints relative to nodes where the robot can be anywhere and
/// the object should be in a stable pose.
/// \code {.py}
/// # transfer path: the box should be fixed wrt the gripper
/// graph.setConstraints (node = 'hold1', grasps = ['grasp_box1',])
/// graph.setConstraints (node = 'hold2', grasps = ['grasp_box2',])
/// \endcode
/// Set constraints corresponding to the robot holding the object. Parameter
/// \c grasps specifies that those constraints (and possibly passive joints)
/// have been defined by the corresponding grasps.
/// \code {.py}
/// # transit path: the box should not move
/// graph.setConstraints (edge = 'move_free_table', lockDof = lockBox)
/// graph.setConstraints (edge = 'move_free_counter', lockDof = lockBox)
/// \endcode
/// Set constraints relative to transit paths, when the robot moves. The box
/// should stay in place.
/// \code {.py}
/// # pre-grasps
/// graph.setConstraints (edge = 'approach_box1_table_e0', lockDof = lockBox)
/// graph.setConstraints (edge = 'approach_box1_table_e1', lockDof = lockBox)
/// graph.setConstraints (node = 'approach_box1_table_n0',
///                       pregrasps = ['pre_grasp_box1',])
/// \endcode
/// Set constraints relative to the node and edges of pre-grasp
/// \c 'pre_grasp_box1': when approaching the box, the box should not move,
/// the approaching position of the gripper is defined by pregrasp
/// \c 'pre_grasp_box1'.
/// \code {.py}
/// graph.setConstraints (edge = 'approach_box1_counter_e0', lockDof = lockBox)
/// graph.setConstraints (edge = 'approach_box1_counter_e1', lockDof = lockBox)
/// graph.setConstraints (node = 'approach_box1_counter_n0',
///                       pregrasps = ['pre_grasp_box1',])
/// graph.setConstraints (edge = 'approach_box2_table_e0', lockDof = lockBox)
/// graph.setConstraints (edge = 'approach_box2_table_e1', lockDof = lockBox)
/// graph.setConstraints (node = 'approach_box2_table_n0',
///                       pregrasps = ['pre_grasp_box2',])
/// graph.setConstraints (edge = 'approach_box2_counter_e0', lockDof = lockBox)
/// graph.setConstraints (edge = 'approach_box2_counter_e1', lockDof = lockBox)
/// graph.setConstraints (node = 'approach_box2_counter_n0',
///                       pregrasps = ['pre_grasp_box2',])
/// \endcode
/// Define three other pre-grasp waypoint edges.
/// \code {.py}
/// graph.setConstraints (edge = 'keep_grasp1', grasps = ['grasp_box1',])
/// graph.setConstraints (edge = 'keep_grasp2', grasps = ['grasp_box2',])
/// \endcode
/// Define constraints on transfer paths. The object is hold in the gripper.
/// Note that parameter \c grasps combined with parameter \c edge adds
/// constraint defined by the grasps and their complements. For axial handles
/// for instance, the rotation around z-axis is free to grasp the object, but
/// once grasped, the rotation is fixed along transfer paths.
/// \sa hpp::manipulation::AxialHandle.
/// \code {.py}
/// res = ps.client.manipulation.problem.applyConstraints \
///       (graph.nodes['free_table'], q_init)
/// if not res[0]:
///   raise Exception ('Init configuration could not be projected.')
/// q_init_proj = res [1]
/// \endcode
/// Define initial configuration by projection \c q_init on sub-manifold
/// corresponding to node \c 'free_table'.
/// \code {.py}
/// res = ps.client.manipulation.problem.applyConstraints \
///       (graph.nodes['free_counter'], q_goal)
/// if not res[0]:
///   raise Exception ('Goal configuration could not be projected.')
/// q_goal_proj = res [1]
/// \endcode
/// Define goal configuration by projecting \c q_goal on sub-manifold
/// corresponding to node \c 'free_counter'.
/// \code
/// r (q_init)
/// \endcode
/// Display initial configuration in viewer.
/// \code {.py}
/// ps.setInitialConfig (q_init_proj)
/// ps.addGoalConfig (q_goal_proj)
/// \endcode
/// Define initial and goal configurations
/// \code {.py}
/// ps.solve ()
/// \endcode
/// Solve problem.
/// \code {.py}
/// pp = PathPlayer (robot.client.basic, r)
/// \endcode
/// Create path player
/// \code {.py}
/// pp (0)
/// \endcode
/// Display solution path.
